#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "hal.h"
#include "hal_i2cslave.h"
#include "ch.h"
#include "errors.h"
#include "i2c.h"
#include "tools.h"
#include "trace.h"
// #include "usb.h"


//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------

#define BUSMEM_BUF_SIZE_LOG2          12U // 4Kbytes

#define BUSMEM_I2C_ADDRESS          0x13U
#define BUSMEM_I2C_BUF_SIZE_LOG2       8U // 256 bytes

// check is this is required
#define STM32_I2C_SLAVE_PRESC          5U
#define STM32_I2C_SLAVE_SCLH          15U
#define STM32_I2C_SLAVE_SCLL           6U
#define STM32_I2C_SLAVE_SDADEL         5U
#define STM32_I2C_SLAVE_SCLDEL         6U

#define MSG_ERROR                      1U
#define MSG_INIT                       2U
#define MSG_WRITE                      3U
#define MSG_READ                       4U

#define BUSMEM_I2C_BUF_TOTAL_SIZE \
   (((1U<<BUSMEM_I2C_BUF_SIZE_LOG2)+sizeof(uint16_t))+sizeof(uint16_t))

//-----------------------------------------------------------------------------
// Type definition
//-----------------------------------------------------------------------------

struct i2c_msg {
   I2CSlaveMsg im_req;
   I2CSlaveMsg im_rsp;
};

struct i2c_engine {
   uint16_t        ie_addr;
   uint8_t *       ie_ptr;
   mailbox_t       ie_mbx;
   struct i2c_msg  ie_i2c_msg;
   uint8_t *       ie_mem_buf;
   size_t          ie_mem_size;
   uint8_t         ie_inbuf[BUSMEM_I2C_BUF_TOTAL_SIZE];
   msg_t           ie_act_msgbufs[4U];
};

//-----------------------------------------------------------------------------
// Forward declarations
//-----------------------------------------------------------------------------

static void _i2c_thread(void * arg);
static void _i2c_handle_request(I2CDriver * i2cp);
static void _i2c_handle_response(I2CDriver * i2cp);
static void _i2c_handle_error(I2CDriver * i2cp);

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------

/** I2C slave configuration */
static const I2CConfig _I2C1_SLAVE_CONFIG = {
  .timingr = ((STM32_I2C_SLAVE_PRESC)  << 28U) |
             ((STM32_I2C_SLAVE_SCLDEL) << 20U) |
             ((STM32_I2C_SLAVE_SDADEL) << 16U) |
             ((STM32_I2C_SLAVE_SCLH) << 8U)    |
             ((STM32_I2C_SLAVE_SCLL) << 0U),
};

/** I2C slave state name */
static const char * _I2C_SLAVE_STATE[] = {
   "I2C_SLV_NONE",
   "I2C_SLV_IDLE",
   "I2C_SLV_RX",
   "I2C_SLV_LCK_RX",
   "I2C_SLV_RSP",
   "I2C_SLV_LCK_RSP",
};

//-----------------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------------

#define BUSMEM_BUF_SIZE        (1U<<BUSMEM_BUF_SIZE_LOG2)

#define BUILD_MSG_ERROR(_err_) (msg_t)((MSG_ERROR<<28U) | ((_err_)&0xFFU))
#define BUILD_MSG_INIT() (msg_t)(MSG_INIT<<28U)
#define BUILD_MSG_WRITE(_addr_, _count_) \
   (msg_t)((MSG_WRITE<<28U) | (((_count_)&0xFFU)<<16U)|((_addr_)&0xFFFFU))
#define BUILD_MSG_READ(_addr_, _count_) \
   (msg_t)((MSG_READ<<28U) | (((_count_)&0xFFU)<<16U)|((_addr_)&0xFFFFU))

#define GET_MSG_SRC(_msg_)   (((uint32_t)(_msg_)) >> 31U)
#define GET_MSG_TYPE(_msg_)  ((((uint32_t)(_msg_)) >> 28U) & 0x7U)
#define GET_MSG_ADDR(_msg_)  ((((uint32_t)(_msg_)) >> 0U) & 0xFFFFU)
#define GET_MSG_COUNT(_msg_) ((((uint32_t)(_msg_)) >> 16U) & 0xFFU)
#define GET_MSG_ERROR(_msg_) ((int)(int8_t)(((uint32_t)(_msg_)) & 0xFFU))

//-----------------------------------------------------------------------------
// Variables
//-----------------------------------------------------------------------------

static THD_WORKING_AREA(_i2c_sandbox, 1024);

static uint8_t busmem_memory[1U<<BUSMEM_BUF_SIZE_LOG2];

static struct i2c_engine _i2c_engine = {
   .ie_mem_buf = &busmem_memory[0],
   .ie_mem_size = sizeof(busmem_memory),
   .ie_i2c_msg =  {
      .im_req = {
         .size = SIZEOF_MEMBER(struct i2c_engine, ie_inbuf),
         .body = &_i2c_engine.ie_inbuf[0],
         .processMsg = &_i2c_handle_request,
         .exception = &_i2c_handle_error,
      },
      .im_rsp = {
         .processMsg = &_i2c_handle_response,
         .exception = &_i2c_handle_error,
      },
   },
};

//-----------------------------------------------------------------------------
// Public API
//-----------------------------------------------------------------------------

void
i2c_init(void)
{
   // message boxes for inter thread/IRQ communication
   chMBObjectInit(&_i2c_engine.ie_mbx, &_i2c_engine.ie_act_msgbufs[0],
                  ARRAY_SIZEOF_MEMBER(struct i2c_engine, ie_act_msgbufs));

   // I2C1 on PA9, PA10
   palSetPadMode(GPIOA, 9U, PAL_MODE_ALTERNATE(4) |
                            PAL_STM32_OTYPE_OPENDRAIN |
                            PAL_STM32_PUPDR_PULLUP);
   palSetPadMode(GPIOA, 10U, PAL_MODE_ALTERNATE(4) |
                             PAL_STM32_OTYPE_OPENDRAIN |
                             PAL_STM32_PUPDR_PULLUP);
}

void
i2c_start(void)
{
   // working thread: handle I2C events
   chThdCreateStatic(_i2c_sandbox, sizeof(_i2c_sandbox),
                     NORMALPRIO, &_i2c_thread, &_i2c_engine);
}

void
i2c_reset(bool irq_context)
{
   if ( irq_context ) {
      (void)chMBPostI(&_i2c_engine.ie_mbx, BUILD_MSG_INIT());
   } else {
      (void)chMBPost(&_i2c_engine.ie_mbx, BUILD_MSG_INIT(), TIME_INFINITE);
   }
}


//-----------------------------------------------------------------------------
// Private implementation
//-----------------------------------------------------------------------------

static void __attribute__((noreturn))
_i2c_thread(void * arg)
{
   struct i2c_engine * eng = (struct i2c_engine *)arg;

   i2cStart(&I2CD1, &_I2C1_SLAVE_CONFIG);
   I2CD1.slaveTimeout = MS2ST(100); // time for complete message

   i2cSlaveConfigure(&I2CD1, &eng->ie_i2c_msg.im_req, &eng->ie_i2c_msg.im_rsp);
   i2cMatchAddress(&I2CD1, BUSMEM_I2C_ADDRESS);

   MSGV("I2C Running");
   for (;;) {
      msg_t msg;
      msg_t rc = chMBFetch(&eng->ie_mbx, &msg, MS2ST(10));

      if ( MSG_OK == rc ) {
         switch ( GET_MSG_TYPE(msg) ) {
            case MSG_ERROR: {
                  unsigned int error = (unsigned int)GET_MSG_ERROR(msg);
                  MSGV("I2C error: %d %s",
                       error, error < ARRAY_SIZE(_I2C_SLAVE_STATE) ?
                           _I2C_SLAVE_STATE[error] : "I2C_UNKNOWN");
               }
               break;
            case MSG_INIT: {
                  MSGV("I2C Power cycle");
               }
               break;
            case MSG_WRITE:
            case MSG_READ: {
                  unsigned int start = GET_MSG_ADDR(msg);
                  unsigned int count = GET_MSG_COUNT(msg);
                  unsigned int end = start + count - 1;
                  bool wr = GET_MSG_TYPE(msg) == MSG_WRITE;
                  if ( count > 1 ) {
                     MSGV("I2C %s> [%02X..%02X]",
                          wr ? ">Wr" : "<Rd", start, end);
                  } else {
                     MSGV("I2C %s [%02X]", wr ? ">Wr" : "<Rd", start);
                  }
               }
               break;
            default:
               MSGV("Unmanaged message: %u", GET_MSG_TYPE(msg));
               break;
         }
      }
   }
}

//-----------------------------------------------------------------------------
// Handlers in IRQ context
//-----------------------------------------------------------------------------

/**
 * Handle a request received from the I2C master.
 */
static void
_i2c_handle_request(I2CDriver * i2cp)
{
   struct i2c_engine * eng = &_i2c_engine;
   size_t size = i2cSlaveBytes(i2cp);

   if ( ! size ) {
      return;
   }

   // we use the first inbuf halfword to store the first byte host master,
   // which contains the address. The MSB (second byte in little-endian
   // encoding) contains the register address the master wants to access
   get_uint16(&eng->ie_addr, eng->ie_inbuf);
   eng->ie_addr &= BUSMEM_BUF_SIZE-1U;

   size_t max_size = BUSMEM_BUF_SIZE-eng->ie_addr;

   if ( size > sizeof(uint16_t) ) {
      // copy received bytes into the main memory, if any
      size -= sizeof(uint16_t);
      if ( max_size < size ) {
         size = max_size;
      }
      memcpy(&eng->ie_mem_buf[eng->ie_addr],
             &eng->ie_i2c_msg.im_req.body[sizeof(uint16_t)],
             size);
      chMBPostI(&eng->ie_mbx, BUILD_MSG_WRITE(eng->ie_addr, size));
   } else {
      chMBPostI(&eng->ie_mbx, BUILD_MSG_READ(eng->ie_addr, size));
   }

   // prepare any I2C read request
   eng->ie_i2c_msg.im_rsp.body = (uint8_t *)&eng->ie_mem_buf[eng->ie_addr];
   eng->ie_i2c_msg.im_rsp.size = max_size;
}

/**
 * Handle the code completion after a response has been sent back to the
 * I2C master.
 */
static void
_i2c_handle_response(I2CDriver * i2cp _unused)
{
   struct i2c_engine * eng = &_i2c_engine;

   eng->ie_i2c_msg.im_rsp.body = NULL;
   eng->ie_i2c_msg.im_rsp.size = 0;
}

/**
 * Handle I2C slave errors.
 */
static void
_i2c_handle_error(I2CDriver * i2cp)
{
   struct i2c_engine * eng = &_i2c_engine;

   eng->ie_addr = 0U;

   uint8_t error = (uint8_t)(i2cSlaveErrors(i2cp)-I2C_UNKNOWN_ERROR);
   chMBPostI(&eng->ie_mbx, BUILD_MSG_ERROR(error));
}
