/**
 * Simple USB-CDC-ACM implementation: USB to UART bridge
 *    for STM32L432KC (Nucleo Board)
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "ch.h"
#include "hal.h"

#include "chprintf.h"

#include "usbcfg.h"
#include "tools.h"

//-----------------------------------------------------------------------------
// Type definitions
//-----------------------------------------------------------------------------

struct forwarder_engine {
   bool fe_resume;
   BaseChannel * fe_usb;
   BaseChannel * fe_serial;
   BaseSequentialStream * fe_dbgch;
   bool fe_update_config;
   bool fe_serial_active;
   cdc_linecoding_t fe_serial_config;
   mutex_t fe_dbgmtx;
};

//-----------------------------------------------------------------------------
// Forward declarations
//-----------------------------------------------------------------------------

bool sdu_requests_hook(USBDriver * usbp);
size_t trace_build_hex(char * dst, size_t dlen,
                       const void * buffer, size_t blen);

static void _forward_usb_to_serial(void * arg);
static void _forward_serial_to_usb(void * arg);
static void _update_line_coding(USBDriver * usbp);

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------

#define FORWARDER_WA_SIZE  THD_WORKING_AREA_SIZE(2048U)

/** Debug port */
static SerialConfig _SD2_CONFIG = {
   .speed = 115200,
};

static const char * _USB_STATES[] = {
   "Uninit",
   "Stop",
   "Ready",
   "Selected",
   "Active",
   "Suspended",
};

//-----------------------------------------------------------------------------
// Variables
//-----------------------------------------------------------------------------

struct forwarder_engine _forwarder_engine = {
   .fe_usb = (BaseChannel *)&SDU1,
   .fe_serial = (BaseChannel *)&SD1,
   .fe_serial_config = {
      .bCharFormat = LC_STOP_1,
      .bParityType = LC_PARITY_NONE,
      .bDataBits = 8,
   },
   .fe_dbgch = (BaseSequentialStream *)&SD2,
};

/** Forward port */
static SerialConfig _sd1_config = {
   .speed = 115200,
   .cr3 = USART_CR3_RTSE | USART_CR3_CTSE,
};

//-----------------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------------

#define MSGV(_fmt_, ...) \
   chMtxLock(&_forwarder_engine.fe_dbgmtx); \
   chprintf(_forwarder_engine.fe_dbgch, _fmt_ "\n", ##__VA_ARGS__); \
   chMtxUnlock(&_forwarder_engine.fe_dbgmtx);

//-----------------------------------------------------------------------------
// Private implementation
//-----------------------------------------------------------------------------

static void
_forward_usb_to_serial(void * arg) {
   struct forwarder_engine * fe = (struct forwarder_engine *)arg;

   for(;fe->fe_resume;) {
      uint8_t buffer[72];
      size_t count;
      count = chnReadTimeout(fe->fe_usb, buffer, ARRAY_SIZE(buffer),
                             OSAL_MS2I(5));
      if ( ! count ) {
         continue;
      }

      //MSGV("U2S> %u", count);

      chnWriteTimeout(fe->fe_serial, buffer, count, TIME_INFINITE);
   }
}

#ifdef _DEBUG_CONFIG
static void
_show_config(struct forwarder_engine * fe)
{
   static char hexbuf[3*sizeof(fe->fe_serial_config)+2];
   trace_build_hex(hexbuf, sizeof(hexbuf),
                   &fe->fe_serial_config, sizeof(fe->fe_serial_config));
   uint32_t baudrate;
   get_uint32(&baudrate, fe->fe_serial_config.dwDTERate);
   MSGV("SET_LINE_CODING: %s, %u bps", hexbuf, baudrate);
   MSGV("speed %u %08x %08x %08x",
        _sd1_config.speed,
        _sd1_config.cr1,
        _sd1_config.cr2,
        _sd1_config.cr3);
}
#endif // _DEBUG_CONFIG

static void
_forward_serial_to_usb(void * arg) {
   struct forwarder_engine * fe = (struct forwarder_engine *)arg;

   for(unsigned int loop=0;fe->fe_resume;) {
      uint8_t buffer[72];
      size_t count;
      count = chnReadTimeout(fe->fe_serial, buffer, ARRAY_SIZE(buffer),
                             OSAL_MS2I(5));
      if ( ! count ) {
         continue;
      }

      chnWriteTimeout(fe->fe_usb, buffer, count, TIME_INFINITE);

      loop++;
   }
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

// Application entry point.
int main(void)
{
   halInit();
   chSysInit();

   struct forwarder_engine * fe = &_forwarder_engine;
   chMtxObjectInit(&fe->fe_dbgmtx);

   // configure USB DP/DM pins
   palSetPadMode(GPIOA, 11, PAL_MODE_ALTERNATE(10)); // DM
   palSetPadMode(GPIOA, 12, PAL_MODE_ALTERNATE(10)); // DP

   set_uint32(fe->fe_serial_config.dwDTERate, _sd1_config.speed);

   // UART2: debug port
   sdStart(&SD2, &_SD2_CONFIG);

   // USB: Serial-over-USB CDC ACM.
   sduObjectInit(&SDU1);
   sduStart(&SDU1, &serusbcfg);

   #pragma clang diagnostic push
   #pragma clang diagnostic ignored "-Wdate-time"
   // only to ensure the new FW has been updated
   MSGV("\nUSB-CDC @ " __TIME__ );
   #pragma clang diagnostic pop

   // Activates the USB driver and then the USB bus pull-up on D+.
   // Note, a delay is inserted in order to not have to disconnect the cable
   // after a reset.
   usbDisconnectBus(serusbcfg.usbp);
   chThdSleepMilliseconds(1500);
   usbStart(serusbcfg.usbp, &usbcfg);
   usbConnectBus(serusbcfg.usbp);

   // thread to forward USB RX packets to UART TX
   thread_t * u2s = NULL;
   // thread to forward UART RX packets to USB TX
   thread_t * s2u = NULL;
   fe->fe_update_config = false;

   for(usbstate_t last_state=USB_UNINIT;;) {
      if ( SDU1.config->usbp->state == last_state && ! fe->fe_update_config ) {
         chThdSleepMilliseconds(100);
         continue;
      }

      last_state = SDU1.config->usbp->state;

      if ( fe->fe_update_config ) {
         // MSGV("Want reconfig");
         fe->fe_resume = false;
      } else {
         MSGV("USB %s", last_state < ARRAY_SIZE(_USB_STATES) ?
                        _USB_STATES[last_state] : "INVALID");

         if ( USB_ACTIVE == last_state ) {
            fe->fe_resume = true;
            if ( ! fe->fe_serial_active ) {
               // UART1: UART master port
               palSetLine(PAL_LINE(GPIOB, 3U));
               palSetLine(PAL_LINE(GPIOB, 6U));
               palSetPadMode(GPIOB, 3, PAL_MODE_OUTPUT_PUSHPULL); // RTS
               palSetPadMode(GPIOB, 6, PAL_MODE_OUTPUT_PUSHPULL); // TX
               palSetPadMode(GPIOB, 4, PAL_MODE_ALTERNATE(7)); // CTS
               palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(7)); // RX
               sdStart(&SD1, &_sd1_config);
               palSetPadMode(GPIOB, 3, PAL_MODE_ALTERNATE(7));
               palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(7));
               fe->fe_serial_active = true;
            }
            if ( ! u2s ) {
               u2s = chThdCreateFromHeap(NULL, FORWARDER_WA_SIZE, "u2s",
                                         NORMALPRIO + 1,
                                         &_forward_usb_to_serial, fe);
            }
            if ( ! s2u ) {
               s2u = chThdCreateFromHeap(NULL, FORWARDER_WA_SIZE, "s2u",
                                         NORMALPRIO + 1,
                                         &_forward_serial_to_usb, fe);
            }
         } else {
            fe->fe_update_config = false;
            fe->fe_resume = false;
         }
      }

      if ( ! fe->fe_resume ) {
         if ( u2s ) {
            chThdWait(u2s);
            u2s = NULL;
         }
         if ( s2u ) {
            chThdWait(s2u);
            s2u = NULL;
         }
         palSetLine(PAL_LINE(GPIOB, 3U));
         palSetLine(PAL_LINE(GPIOB, 6U));
         palSetPadMode(GPIOB, 3, PAL_MODE_OUTPUT_PUSHPULL); // RTS
         palSetPadMode(GPIOB, 6, PAL_MODE_OUTPUT_PUSHPULL); // TX
         sdStop(&SD1);
         fe->fe_serial_active = false;
         if ( fe->fe_update_config ) {
            get_uint32(&_sd1_config.speed, fe->fe_serial_config.dwDTERate);
            #if 1
            _sd1_config.cr2 &= ~USART_CR2_STOP_Msk;
            uint8_t bitcount;
            char parity = ' ';
            switch ( fe->fe_serial_config.bCharFormat ) {
               case LC_STOP_1:
                  _sd1_config.cr2 |= USART_CR2_STOP1_BITS;
                  break;
               case LC_STOP_1P5:
                  _sd1_config.cr2 |= USART_CR2_STOP1P5_BITS;
                  break;
               case LC_STOP_2:
                  _sd1_config.cr2 |= USART_CR2_STOP2_BITS;
                  break;
               default:
                  MSGV("Invalid stop bits value: %u",
                       fe->fe_serial_config.bCharFormat);
                  break;
            }
            _sd1_config.cr1 &= ~(USART_CR1_M1_Msk|
                                 USART_CR1_M0_Msk|
                                 USART_CR1_PCE_Msk|
                                 USART_CR1_PS_Msk);
            switch ( fe->fe_serial_config.bParityType ) {
               case LC_PARITY_NONE:
                  bitcount = 0;
                  parity = 'N';
                  break;
               case LC_PARITY_ODD:
                  bitcount = 1;
                  parity = 'O';
                  _sd1_config.cr1 |= USART_CR1_PCE|USART_CR1_PS;
                  break;
               case LC_PARITY_EVEN:
                  parity = 'E';
                  bitcount = 1;
                  _sd1_config.cr1 |= USART_CR1_PCE;
                  break;
               case LC_PARITY_MARK:
               case LC_PARITY_SPACE:
               default:
                  bitcount = 0;
                  // mark and space could be supported, but those modes are
                  // barely used nowadays, so skip the mess to handle them
                  MSGV("Unsupported parity mode: %u",
                       fe->fe_serial_config.bParityType);
                  break;
            }
            switch ( fe->fe_serial_config.bDataBits ) {
               case 7:
               case 8:
                  bitcount += fe->fe_serial_config.bDataBits;
                  break;
               default:
                  MSGV("Unsupported bit count: %u",
                       fe->fe_serial_config.bDataBits);
                  break;
            }
            if ( bitcount == 7 ) {
               _sd1_config.cr1 |= USART_CR1_M1;
            } else if ( bitcount == 9 ) {
               _sd1_config.cr1 |= USART_CR1_M0;
            }

            MSGV("UART1: %u %u%c%u",
                 _sd1_config.speed,
                 fe->fe_serial_config.bDataBits,
                 parity,
                 fe->fe_serial_config.bCharFormat & 1 ?
                     5 : 1+(fe->fe_serial_config.bCharFormat>>1));
            #endif
            fe->fe_update_config = false;
            fe->fe_resume = true;
            // force reinit
            last_state = USB_UNINIT;
         }
      }
   }
}

/**
 * Format the content of a binary buffer as hexadecimal values.
 *
 * @note Destination buffer should be 3* input buffer size + 2 minimum to fit
 * all characters
 *
 * @param[in,out] dst destination string to update
 * @param[in] dlen maximum size ot the destination string
 * @param[in] buffer the buffer to dump
 * @param[in] blen the number of bytes to dump
 * @return the actual count of char in the destination buffer
 */
size_t
trace_build_hex(char * dst, size_t dlen, const void * buffer, size_t blen)
{
   char * hexp = &dst[0];
   ssize_t rem = (ssize_t)dlen;
   for (unsigned int ix=0; ix<blen; ++ix) {
      int count = snprintf(hexp, (size_t)MAX(0, rem),
                           "%02x ", ((const uint8_t*)buffer)[ix]);
      if ( count >= rem ) {
         break;
      }
      rem -= count;
      hexp += count;
   }
   if ( hexp > dst ) {
      hexp--;
   }
   *hexp++ = '\0';

   return (size_t)(hexp-dst);
}

bool
sdu_requests_hook(USBDriver * usbp)
{
   if ((usbp->setup[0] & USB_RTYPE_TYPE_MASK) == USB_RTYPE_TYPE_CLASS) {
      switch (usbp->setup[1]) {
         case CDC_GET_LINE_CODING:
            usbSetupTransfer(usbp,
                             (uint8_t *)&_forwarder_engine.fe_serial_config,
                             sizeof(_forwarder_engine.fe_serial_config), NULL);
            return true;
         case CDC_SET_LINE_CODING:
            usbSetupTransfer(usbp,
                             (uint8_t *)&_forwarder_engine.fe_serial_config,
                             sizeof(_forwarder_engine.fe_serial_config),
                             &_update_line_coding);
            return true;
         case CDC_SET_CONTROL_LINE_STATE:
            /* Nothing to do, there are no control lines.*/
            usbSetupTransfer(usbp, NULL, 0, NULL);
            return true;
         default:
            return false;
      }
   }
   return false;
}

static void
_update_line_coding(USBDriver * usbp _unused)
{
   _forwarder_engine.fe_update_config = true;
}
