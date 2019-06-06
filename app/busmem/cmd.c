/**
 * Remote command handler
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include "hal.h"
#include "ch.h"
#include "cmd.h"
#include "errors.h"
#include "tools.h"
#include "trace.h"
#include "usb.h"
#include "busmem_gitbldver.h"  // automatically generated in build directory

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------

#define BUSMEM_MAJOR 0
#define BUSMEM_MINOR 1
#define BUSMEM_PATCH 0

/** Base identifier for command identifier over communication link */
#define CMD_BASE  '@'

//-----------------------------------------------------------------------------
// Type definitions
//-----------------------------------------------------------------------------

/** Command state machine states */
enum cmd_sm_state {
   SS_IDLE,       /**< State machine is not ready to do anything */
   SS_RX_CMD,     /**< Ready to receive a command */
   SS_RX_ARG,     /**< Ready to receive argument */
   SS_REQUEST,    /**< A full request has been received */
   SS_TX,         /**< Send reply */
   SS_ERROR,      /**< On error */
};

/** Command communication data frame */
struct cmd_frame {
   uint8_t uf_cmd;     /**< Received command */
   uint8_t uf_length;  /**< Received length */
   union {
      uint8_t uf_payload[CMD_FRAME_SIZE]; /*<Binary payload buf */
      char uf_message[CMD_FRAME_SIZE]; /**< ASCII payload buf */
   };
};

/** Command state machine */
struct cmd_sm {
   enum cmd_sm_state cs_state; /**< Current state machine state */
   uint8_t cs_pos;             /**< Current position in payload buffer */
   uint8_t cs_step;            /**< Arbitrary response step for commands */
   int8_t  cs_error;           /**< Last error code */
   bool    cs_can_tx;          /**< Send to host is safe */
   mutex_t cs_tx_mtx;          /**< Exclusive access to TX-to-host */
   struct cmd_frame cs_frame;  /**< Data frame */
};

/**
 * Command handler.
 *
 * @param[in,out] sm command state machine
 * @return @c ENOERR on success, negative POSIX error code on error,
 *            positive POSIX error code on deferred reply
 */
typedef int (*cmd_handler_t)(struct cmd_sm * sm);

//-----------------------------------------------------------------------------
// Forward declarations
//-----------------------------------------------------------------------------

static cmd_handler_t _cmd_get_handler(uint8_t cmdcode);
static int _cmd_error(struct cmd_sm * sm);
static int _cmd_help(struct cmd_sm * sm);
static int _cmd_ident(struct cmd_sm * sm);
static void _cmd_inject_command(struct cmd_sm * sm);

//-----------------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------------

/** Convert a command code into a slot. Not protected against invalid value */
#define CMD_SLOT(_code_) ((_code_) - CMD_BASE)

//-----------------------------------------------------------------------------
// Constants (continued)
//-----------------------------------------------------------------------------

/**
 * Command dispatcher array.
 * There can be up to 26 handlers, one for each alphabet character
 */
static const cmd_handler_t _CMD_COMMANDS[] = {
   [CMD_SLOT('@')] = &_cmd_error,
   [CMD_SLOT('H')] = &_cmd_help,
   [CMD_SLOT('I')] = &_cmd_ident,
};

/** List of commands that do not complete immediately */
static const bool _CMD_ASYNC_COMMANDS[] = {
   [CMD_SLOT('@')] = false,
};

//-----------------------------------------------------------------------------
// Variables
//-----------------------------------------------------------------------------

/** The unique instance of the UART state machine */
static struct cmd_sm _cmd_sm;

//-----------------------------------------------------------------------------
// Static inline
//-----------------------------------------------------------------------------

/**
 * Tells wether a command code maps to a slot or not
 */
static inline bool
_cmd_is_valid_cmd_slot(uint8_t code)
{
   return (code >= CMD_BASE) &&
          (code <= (CMD_BASE + ARRAY_SIZE(_CMD_COMMANDS)));
}

/**
 * Map a command code (defined as an ASCII letter) to a handler slot.
 * @note code is not checked for validity, use #_cmd_is_valid_cmd_slot first
 */
static inline unsigned int
_cmd_get_cmd_slot(uint8_t code)
{
   return CMD_SLOT(code);
}

/**
 * Tell if a command completes asynchronously
 *
 * @return @c true if the command completes asynchronously
 */
static inline bool
_cmd_is_async(uint8_t code)
{
   if ( (code >= CMD_BASE) &&
        (code <= (CMD_BASE + ARRAY_SIZE(_CMD_ASYNC_COMMANDS))) ) {
      return _CMD_ASYNC_COMMANDS[CMD_SLOT(code)];
   }

   // if command is not found (it should not...) considers it synchronous
   return false;
}


//-----------------------------------------------------------------------------
// Public API
//-----------------------------------------------------------------------------

void
cmd_init(void)
{
   chMtxObjectInit(&_cmd_sm.cs_tx_mtx);
}

/**
 * Reset the communication state machine
 *
 * @context UART RX IRQ & main
 * @param[in,out] sm the state machine
 */
void
cmd_reset(void)
{
   struct cmd_sm * sm = &_cmd_sm;
   // if any transmission is on-going, cancels it immediately. The outgoing
   // reply is simply aborted

   // clear out the dataframe content
   sm->cs_frame.uf_length = 0;
   sm->cs_pos = 0;
   sm->cs_error = 0;

   // now ready to receive the first command
   sm->cs_state = SS_RX_CMD;
}

/**
 * Inject a new received byte into the state machine
 *
 * @context USB thread
 * @param[in] byte
 */
void
cmd_receive_byte(uint8_t byte)
{
   struct cmd_sm * sm = &_cmd_sm;

   // echo incoming byte
   if ( true ) {
      chMtxLock(&_cmd_sm.cs_tx_mtx);
      usb_send_reply(&byte, sizeof(byte));
      chMtxUnlock(&_cmd_sm.cs_tx_mtx);
   }

   switch (sm->cs_state) {
      case SS_RX_CMD: // ready to receive a command
         sm->cs_frame.uf_cmd = byte;
         sm->cs_state = SS_RX_ARG;
         break;
      case SS_RX_ARG: // ready to receive the separator
         if ( (byte == (uint8_t)' ') || (byte == (uint8_t)':') ) {
            // ignore
            break;
         }
         if ( byte == (uint8_t)'\n' ) {
            sm->cs_state = SS_REQUEST;
            _cmd_inject_command(sm);
            break;
         }
         if ( sm->cs_frame.uf_length >= sizeof(sm->cs_frame.uf_payload) ) {
            sm->cs_error = -E2BIG;
            sm->cs_state = SS_ERROR;
            break;
         }
         if ( isalpha(byte) || isdigit(byte) ) {
            sm->cs_frame.uf_payload[sm->cs_frame.uf_length++] = (char)byte;
         } else {
            sm->cs_error = -EINVAL;
            sm->cs_state = SS_ERROR;
         }
         break;
      case SS_ERROR: // only accept LF to reset the RX state machine
         if ( byte == (uint8_t)'\n' ) {
            sm->cs_state = SS_REQUEST;
            sm->cs_frame.uf_cmd = '@'; // special error case
            _cmd_inject_command(sm);
            break;
         }
      case SS_TX:
      case SS_IDLE:       // not ready to receive any byte
      case SS_REQUEST:    // already processing a received request
      default:
         // we do not accept for bytes for now, ignore them if the remote peer
         // keep pushing them although we do not have sent a response. If it
         // gets out of sync, the comm. protocol specifies it should trigger
         // a break sequence so we can reset our comm. state machine anyway
         break;
   }
}

/**
 * Push stream data to the communication link, if possible: if a command
 * handling is on-going, the buffer is simply discarded.
 */
void
cmd_stream_msg(const uint8_t * buf, size_t length)
{
   if ( ! chMtxTryLock(&_cmd_sm.cs_tx_mtx) ) {
      return;
   }

   if ( _cmd_sm.cs_can_tx ) {
      usb_send_reply(buf, (uint8_t)length);
   }

   chMtxUnlock(&_cmd_sm.cs_tx_mtx);
}

/**
 * Signal explicit end-of-streaming
 */
void
cmd_signal_eos(void)
{
   const uint8_t buffer[] = "\n";
   cmd_stream_msg(buffer, sizeof(uint8_t));
   _cmd_sm.cs_can_tx = false;
}

/**
 * Signal the USB status
 *
 * @param[in] available whether communication with host is possible.
 */
void
cmd_signal_usb(bool available _unused)
{

}

//-----------------------------------------------------------------------------
// Private Implementation
//-----------------------------------------------------------------------------

/**
 * Inject a command into the command state machine
 *
 * @context main
 * @param[in,out] sm the state machine
 */
static void
_cmd_inject_command(struct cmd_sm * sm)
{
   uint8_t cmd = sm->cs_frame.uf_cmd;
   cmd_handler_t handler = _cmd_get_handler(cmd);

   // compute the communication CRC
   sm->cs_state = SS_TX;

   sm->cs_step = 0U;

   int rc;
   bool send_ack = true;
   char buffer[8];

   chMtxLock(&_cmd_sm.cs_tx_mtx);

   for(;;) {
      // command is valid, we can now invoke it
      // it is mandatory that uf_payload is 16-bit (word) aligned
      if ( ! handler ) {
         rc = -ENOSYS;
      } else {
         rc = (*handler)(sm);
      }
      if ( send_ack ) {
         int len;
         if ( rc ) {
            len = snprintf(buffer, sizeof(buffer), "E %d\n", rc);
         } else {
            len = snprintf(buffer, sizeof(buffer),
                          "%c 0\n", sm->cs_frame.uf_cmd |= 0x20);
         }
         usb_send_reply((const uint8_t*)buffer,
                        MIN((uint8_t)len, (uint8_t)sizeof(buffer)));
         send_ack = false;
      }
      if ( rc ) {
         break;
      }
      if ( sm->cs_frame.uf_length ) {
         usb_send_reply(sm->cs_frame.uf_payload, sm->cs_frame.uf_length);
         sm->cs_step += 1U;
      } else {
         if ( ! _cmd_is_async(cmd) ) {
            // end of response
            buffer[0] = (uint8_t)'\n';
            usb_send_reply((const uint8_t*)buffer, sizeof(uint8_t));
         }
         break;
      }
   }

   chMtxUnlock(&_cmd_sm.cs_tx_mtx);

   cmd_reset();
}

/**
 * Dispatch a command code into a command handler, if any found
 *
 * @context main
 * @param[in] cmdcode the code of the command
 * @return command handler to invoke, or NULL if not found
 */
static cmd_handler_t
_cmd_get_handler(uint8_t cmdcode)
{
   // check if the command code is valid
   if ( ! _cmd_is_valid_cmd_slot(cmdcode) ) {
      return NULL;
   }

   // retrieve the command handler
   return _CMD_COMMANDS[_cmd_get_cmd_slot(cmdcode)];
}

static int
_cmd_error(struct cmd_sm * sm)
{
   return (sm->cs_error <= 0) ? sm->cs_error : -sm->cs_error;
}

static int
_cmd_help(struct cmd_sm * sm)
{
   struct cmd_frame * cf = &sm->cs_frame;
   char * p;

   switch ( sm->cs_step ) {
      case 0U:
         p = strncpy(cf->uf_message,
                     "H:          Help\n",
                     sizeof(cf->uf_message));
         break;
      case 1U:
         p = strncpy(cf->uf_message,
                     "I:          Identify\n",
                     sizeof(cf->uf_message));
         break;
      default:
         p = NULL;
         break;
   }

   if ( p ) {
      sm->cs_frame.uf_length = (uint8_t)strlen(p);
   } else {
      sm->cs_frame.uf_length = 0U;
   }

   return ENOERR;
}

/**
 * Identify the firmware
 *
 * @return @c ENOERR on succes or a negative POSIX error
 */
static int
_cmd_ident(struct cmd_sm * sm)
{
   struct cmd_frame * cf = &sm->cs_frame;
   int len;

   if ( ! sm->cs_step ) {
      len = snprintf(cf->uf_message, sizeof(cf->uf_message),
                    "i: "
      #ifdef DEBUG
                    "d"
      #else // DEBUG
                    "r"
      #endif // !DEBUG
                    "%u.%u.%u-"  BUSMEM_GITVER " (%u)\n",
                    BUSMEM_MAJOR, BUSMEM_MINOR, BUSMEM_PATCH,
                    (unsigned int)GIT_REFERENCE(date)());
   } else {
      len = 0;
   }

   sm->cs_frame.uf_length = (uint8_t)len;

   return ENOERR;
}

