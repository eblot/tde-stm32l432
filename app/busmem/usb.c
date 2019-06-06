/**
 * USB CDC communications
 */

#include "hal.h"
#include "tools.h"
#include "trace.h"
#include "cmd.h"
#include "usb.h"

//-----------------------------------------------------------------------------
// Debug configuration
//-----------------------------------------------------------------------------

// show TX/RX communication with host
#define SHOW_COMM_FRAME
// show USB SM transitions
#define SHOW_USB_STATE

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------

/** Endpoints to be used for USBD1. */
#define USBD1_DATA_REQUEST_EP           1U
#define USBD1_DATA_AVAILABLE_EP         1U
#define USBD1_INTERRUPT_REQUEST_EP      2U

#define USBD_CDC_ACM_FEATURE_REQUESTS   (1U<<0U)
#define USBD_CDC_ACM_LINE_REQUESTS      (1U<<1U)
#define USBD_CDC_ACM_SENDBREAK_REQUESTS (1U<<2U)
#define USBD_CDC_ACM_NOTIFY_REQUESTS    (1U<<3U)

/** USB Device Descriptor. */
static const uint8_t _VCOM_DEVICE_DESCRIPTOR_DATA[18] = {
  USB_DESC_DEVICE       (0x0110,        /* bcdUSB (1.1).                    */
                         0x02,          /* bDeviceClass (CDC).              */
                         0x00,          /* bDeviceSubClass.                 */
                         0x00,          /* bDeviceProtocol.                 */
                         0x40,          /* bMaxPacketSize.                  */
                         0x0483,        /* idVendor (ST).                   */
                         0x5740,        /* idProduct.                       */
                         0x0200,        /* bcdDevice.                       */
                         1,             /* iManufacturer.                   */
                         2,             /* iProduct.                        */
                         3,             /* iSerialNumber.                   */
                         1)             /* bNumConfigurations.              */
};

/* Configuration Descriptor tree for a CDC.*/
static const uint8_t _VCOM_CONFIGURATION_DESCRIPTOR_DATA[67] = {
  /* Configuration Descriptor.*/
  USB_DESC_CONFIGURATION(67,            /* wTotalLength.                    */
                         0x02,          /* bNumInterfaces.                  */
                         0x01,          /* bConfigurationValue.             */
                         0,             /* iConfiguration.                  */
                         0xC0,          /* bmAttributes (self powered).     */
                         50),           /* bMaxPower (100mA).               */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x00,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x01,          /* bNumEndpoints.                   */
                         0x02,          /* bInterfaceClass (Communications
                                           Interface Class, CDC section
                                           4.2).                            */
                         0x02,          /* bInterfaceSubClass (Abstract
                                         Control Model, CDC section 4.3).   */
                         0x01,          /* bInterfaceProtocol (AT commands,
                                           CDC section 4.4).                */
                         0),            /* iInterface.                      */
  /* Header Functional Descriptor (CDC section 5.2.3).*/
  USB_DESC_BYTE         (5),            /* bLength.                         */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x00),         /* bDescriptorSubtype (Header
                                           Functional Descriptor.           */
  USB_DESC_BCD          (0x0110),       /* bcdCDC.                          */
  /* Call Management Functional Descriptor. */
  USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x01),         /* bDescriptorSubtype (Call Management
                                           Functional Descriptor).          */
  USB_DESC_BYTE         (0x00),         /* bmCapabilities (D0+D1).          */
  USB_DESC_BYTE         (0x01),         /* bDataInterface.                  */
  /* ACM Functional Descriptor.*/
  USB_DESC_BYTE         (4),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x02),         /* bDescriptorSubtype (Abstract
                                           Control Management Descriptor).  */
                                        /* bmCapabilities.                  */
  USB_DESC_BYTE         (USBD_CDC_ACM_LINE_REQUESTS|
                         USBD_CDC_ACM_SENDBREAK_REQUESTS),
  /* Union Functional Descriptor.*/
  USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x06),         /* bDescriptorSubtype (Union
                                           Functional Descriptor).          */
  USB_DESC_BYTE         (0x00),         /* bMasterInterface (Communication
                                           Class Interface).                */
  USB_DESC_BYTE         (0x01),         /* bSlaveInterface0 (Data Class
                                           Interface).                      */
  /* Endpoint 2 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD1_INTERRUPT_REQUEST_EP|0x80,
                         0x03,          /* bmAttributes (Interrupt).        */
                         0x0008,        /* wMaxPacketSize.                  */
                         0xFF),         /* bInterval.                       */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x01,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x02,          /* bNumEndpoints.                   */
                         0x0A,          /* bInterfaceClass (Data Class
                                           Interface, CDC section 4.5).     */
                         0x00,          /* bInterfaceSubClass (CDC section
                                           4.6).                            */
                         0x00,          /* bInterfaceProtocol (CDC section
                                           4.7).                            */
                         0x00),         /* iInterface.                      */
  /* Endpoint 3 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD1_DATA_AVAILABLE_EP,       /* bEndpointAddress.*/
                         0x02,          /* bmAttributes (Bulk).             */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x00),         /* bInterval.                       */
  /* Endpoint 1 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD1_DATA_REQUEST_EP|0x80,    /* bEndpointAddress.*/
                         0x02,          /* bmAttributes (Bulk).             */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x00)          /* bInterval.                       */
};

/** U.S. English language identifier. */
static const uint8_t _VCOM_STRING0[] = {
  USB_DESC_BYTE(4),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  USB_DESC_WORD(0x0409)                 /* wLANGID (U.S. English).          */
};

/** Vendor string. */
static const uint8_t _VCOM_STRING1[] = {
  USB_DESC_BYTE(14),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'I', 0, 'r', 0, 'o', 0, 'a', 0, 'z', 0, 'h', 0
};

/*
 * Device Description string.
 */
static const uint8_t _VCOM_STRING2[] = {
  USB_DESC_BYTE(46),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'D', 0, 'u', 0, 'm', 0, 'm', 0, 'y', 0, ' ', 0, 'B', 0, 'u', 0,
  's', 0, ' ', 0, 'S', 0, 'l', 0, 'a', 0, 'v', 0, 'e', 0, ' ', 0,
  'M', 0, 'e', 0, 'm', 0, 'o', 0, 'r', 0, 'y', 0,
};

/** Serial Number string. */
static const uint8_t _VCOM_STRING3[] = {
  USB_DESC_BYTE(12),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  // serial number is often used to number the CDC ACM port on hosts, so try
  // to make it remarkable
  '0', 0, '0', 0, '0', 0, '0', 0, '3', 0,
};

/** Configuration Descriptor wrapper. */
static const USBDescriptor _VCOM_CONFIGURATION_DESCRIPTOR = {
  sizeof(_VCOM_CONFIGURATION_DESCRIPTOR_DATA),
  _VCOM_CONFIGURATION_DESCRIPTOR_DATA
};

/** Device Descriptor wrapper. */
static const USBDescriptor _VCOM_DEVICE_DESCRIPTOR = {
  sizeof(_VCOM_DEVICE_DESCRIPTOR_DATA), _VCOM_DEVICE_DESCRIPTOR_DATA
};

/** Strings wrappers array. */
static const USBDescriptor _VCOM_STRINGS[] = {
  {sizeof(_VCOM_STRING0), _VCOM_STRING0},
  {sizeof(_VCOM_STRING1), _VCOM_STRING1},
  {sizeof(_VCOM_STRING2), _VCOM_STRING2},
  {sizeof(_VCOM_STRING3), _VCOM_STRING3}
};

#ifdef SHOW_USB_STATE
static const char * _USB_STATES[] = {
   "Uninit",
   "Stop",
   "Ready",
   "Selected",
   "Active",
   "Suspended",
};
# define MSG_USB_STATE(_st_) \
   MSGV("USB %s", \
        (_st_) < ARRAY_SIZE(_USB_STATES) ? _USB_STATES[(_st_)] : "INVALID");
#else // SHOW_USB_STATE
# define MSG_USB_STATE(_st_)
#endif // !SHOW_USB_STATE

enum usb_debug_message {
   USB_DBG_NONE,
   USB_DBG_BREAK,
};

//-----------------------------------------------------------------------------
// Type definitions
//-----------------------------------------------------------------------------

struct usb_engine {
   SerialUSBDriver ue_sdu;           /**< Virtual serial port over USB*/
   USBInEndpointState ue_ep1in;      /**< IN EP1 state */
   USBOutEndpointState ue_ep1out;    /**< OUT EP1 state */
   USBInEndpointState ue_ep2in;      /**< IN EP2 state */
   cdc_linecoding_t ue_lineconf;     /**< Serial line configuration */
};

//-----------------------------------------------------------------------------
// Forward declarations
//-----------------------------------------------------------------------------

static void _usb_thread(void * arg);
static const USBDescriptor * _usb_get_descriptor(
   USBDriver *usbp, uint8_t dtype, uint8_t dindex, uint16_t lang);
static void _usb_event(USBDriver * usbp, usbevent_t event);
static void _usb_sof_handler(USBDriver * usbp);
static bool _usb_requests_hook(USBDriver * usbp);

//-----------------------------------------------------------------------------
// Variables
//-----------------------------------------------------------------------------

static struct usb_engine _usb_engine = {
   .ue_lineconf = {
      .bCharFormat = LC_STOP_1,
      .bParityType = LC_PARITY_NONE,
      .bDataBits = 8,
   },
};

static THD_WORKING_AREA(_usb_sandbox, 2048);

//-----------------------------------------------------------------------------
// Constants (continued)
//-----------------------------------------------------------------------------

/** EP1 initialization structure (both IN and OUT). */
static const USBEndpointConfig ep1config = {
  .ep_mode = USB_EP_MODE_TYPE_BULK,
  .setup_cb = NULL,
  .in_cb = &sduDataTransmitted,
  .out_cb = &sduDataReceived,
  .in_maxsize = 0x0040,
  .out_maxsize = 0x0040,
  .in_state = &_usb_engine.ue_ep1in,
  .out_state = &_usb_engine.ue_ep1out,
  .ep_buffers = 2,
  .setup_buf = NULL,
};

/** EP2 initialization structure (IN only). */
static const USBEndpointConfig ep2config = {
  .ep_mode = USB_EP_MODE_TYPE_INTR,
  .setup_cb = NULL,
  .in_cb = &sduInterruptTransmitted,
  .out_cb = NULL,
  .in_maxsize = 0x0010,
  .out_maxsize = 0x0000,
  .in_state = &_usb_engine.ue_ep2in,
  .out_state = NULL,
  .ep_buffers = 1,
  .setup_buf = NULL,
};

/** USB driver configuration. */
static const USBConfig _USBCFG = {
  .event_cb = &_usb_event,
  .get_descriptor_cb = &_usb_get_descriptor,
  .requests_hook_cb = &_usb_requests_hook,
  .sof_cb = &_usb_sof_handler,
};

/** Serial over USB driver configuration. */
static const SerialUSBConfig _SDU1_CFG = {
   .usbp = &USBD1,
   .bulk_in = USBD1_DATA_REQUEST_EP,
   .bulk_out = USBD1_DATA_AVAILABLE_EP,
   .int_in = USBD1_INTERRUPT_REQUEST_EP
};

//-----------------------------------------------------------------------------
// Public API
//-----------------------------------------------------------------------------

void
usb_init(void)
{
   palSetPadMode(GPIOA, 11U, PAL_MODE_ALTERNATE(10)); // DM
   palSetPadMode(GPIOA, 12U, PAL_MODE_ALTERNATE(10)); // DP
}

void
usb_start(void)
{
   // USB: Serial-over-USB CDC ACM.
   sduObjectInit(&_usb_engine.ue_sdu);
   sduStart(&_usb_engine.ue_sdu, &_SDU1_CFG);

   // working thread: handle USB events
   chThdCreateStatic(_usb_sandbox, sizeof(_usb_sandbox),
                     NORMALPRIO, &_usb_thread, NULL);
}

void
usb_send_reply(const uint8_t * buffer, uint8_t size)
{
   #ifdef SHOW_COMM_FRAME
   char hexbuf[3*CMD_FRAME_SIZE+2];
   trace_hex(hexbuf, sizeof(hexbuf), buffer, size);
   MSGV("USB: W>: %s", hexbuf);
   #endif // SHOW_COMM_FRAME

   chnWriteTimeout(&_usb_engine.ue_sdu, buffer, size, TIME_INFINITE);
}

void
usb_debug_msg(uint8_t category, uint16_t value)
{
   switch ( category ) {
      case USB_DBG_BREAK:
         MSGV("USB: Break for %d 0x%04x", value, value);
         break;
      default:
         MSGV("USB: Debug %u: 0x%04x", category, value);
         break;
   }
}

//-----------------------------------------------------------------------------
// Private implementation
//-----------------------------------------------------------------------------

static void __attribute__((noreturn))
_usb_thread(void * arg _unused)
{
   chRegSetThreadName("usb");

   #ifdef SHOW_USB_STATE
   MSGV("USB Started");
   #endif // SHOW_USB_STATE

   // Activates the USB driver and then the USB bus pull-up on D+.
   // Note, a delay is inserted in order to not have to disconnect the cable
   // after a reset.
   USBDriver * usb = _SDU1_CFG.usbp;
   usbDisconnectBus(usb);
   chThdSleepMilliseconds(1500);
   usbStart(usb, &_USBCFG);
   usbConnectBus(usb);

   BaseChannel * usbch = (BaseChannel *)&_usb_engine.ue_sdu;

   for(usbstate_t last_state=USB_UNINIT;;) {
      usbstate_t state = _usb_engine.ue_sdu.config->usbp->state;

      if ( USB_ACTIVE == state ) {
         if ( state != last_state ) {
            MSG_USB_STATE(state);
            cmd_signal_usb(true);
            last_state = state;
         }

         uint8_t buffer[CMD_FRAME_SIZE];
         size_t count;
         count = chnReadTimeout(usbch, buffer, ARRAY_SIZE(buffer), MS2ST(10));
         if ( ! count ) {
            continue;
         }

         #ifdef SHOW_COMM_FRAME
         char hexbuf[3*CMD_FRAME_SIZE+2];
         trace_hex(hexbuf, sizeof(hexbuf), buffer, count);
         MSGV("USB: R<: %s", hexbuf);
         #endif // SHOW_COMM_FRAME

         for(uint8_t pos=0; pos<count; pos++) {
            cmd_receive_byte(buffer[pos]);
         }
      } else {
         if (state == last_state ) {
            chThdSleepMilliseconds(10);
            continue;
         }

         MSG_USB_STATE(state);
         cmd_signal_usb(false);
         cmd_reset();
         last_state = state;
      }
   }
}

/**
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */
static const USBDescriptor *
_usb_get_descriptor(USBDriver *usbp _unused, uint8_t dtype, uint8_t dindex,
                   uint16_t lang _unused)
{
   switch (dtype) {
      case USB_DESCRIPTOR_DEVICE:
         return &_VCOM_DEVICE_DESCRIPTOR;
      case USB_DESCRIPTOR_CONFIGURATION:
         return &_VCOM_CONFIGURATION_DESCRIPTOR;
      case USB_DESCRIPTOR_STRING:
         if (dindex < 4) {
            return &_VCOM_STRINGS[dindex];
         }
         return NULL;
      default:
         return NULL;
   }
}

/**
 * Handles the USB driver global events.
 */
static void
_usb_event(USBDriver * usbp, usbevent_t event)
{
   switch (event) {
      case USB_EVENT_ADDRESS:
         break;
      case USB_EVENT_CONFIGURED:
         chSysLockFromISR();
         // Enables the endpoints specified into the configuration.
         // Note, this callback is invoked from an ISR so I-Class functions
         // must be used.
         usbInitEndpointI(usbp, USBD1_DATA_REQUEST_EP, &ep1config);
         usbInitEndpointI(usbp, USBD1_INTERRUPT_REQUEST_EP, &ep2config);
         // Resetting the state of the CDC subsystem.*/
         sduConfigureHookI(&_usb_engine.ue_sdu);
         chSysUnlockFromISR();
         break;
      case USB_EVENT_RESET:
         /* Falls into.*/
      case USB_EVENT_UNCONFIGURED:
         /* Falls into.*/
      case USB_EVENT_SUSPEND:
         chSysLockFromISR();
         // Disconnection event on suspend.
         sduSuspendHookI(&_usb_engine.ue_sdu);
         chSysUnlockFromISR();
         break;
      case USB_EVENT_WAKEUP:
         chSysLockFromISR();
         // Disconnection event on suspend.
         sduWakeupHookI(&_usb_engine.ue_sdu);
         chSysUnlockFromISR();
         break;
      case USB_EVENT_STALLED:
      default:
         break;
   }
}

/**
 * Handleds Start Of Frame.
 */
static void _usb_sof_handler(USBDriver * usbp _unused)
{
   osalSysLockFromISR();
   sduSOFHookI(&_usb_engine.ue_sdu);
   osalSysUnlockFromISR();
}

static bool
_usb_requests_hook(USBDriver * usbp)
{
   if ((usbp->setup[0] & USB_RTYPE_TYPE_MASK) == USB_RTYPE_TYPE_CLASS) {
      switch (usbp->setup[1]) {
         case CDC_GET_LINE_CODING:
            usbSetupTransfer(usbp,
                             (uint8_t *)&_usb_engine.ue_lineconf,
                             sizeof(_usb_engine.ue_lineconf), NULL);
            return true;
         case CDC_SET_LINE_CODING:
            usbSetupTransfer(usbp,
                             (uint8_t *)&_usb_engine.ue_lineconf,
                             sizeof(_usb_engine.ue_lineconf), NULL);
            return true;
         case CDC_SET_CONTROL_LINE_STATE:
            /* Nothing to do, there are no control lines.*/
            usbSetupTransfer(usbp, NULL, 0, NULL);
            return true;
         case CDC_SEND_BREAK:
            // cannot call MSGV from IRQ context, post a debug message
            // to get called back from the debug thread
            if ( (usbp->setup[2]|(usbp->setup[3]<<8U)) > 0 ) {
               // 0 means "end of break" for a duration-less sendbreak request
               // moreover, the host may send a end-of-break condition when
               // it releases its comm port, and we do not want to reset
               // registers when the host leaves the party.
               //mx_reset(true);
            }
            return true;
         default:
            return false;
      }
   }
   return false;
}
