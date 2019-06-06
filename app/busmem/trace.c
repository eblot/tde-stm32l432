/**
 * Debug trace
 */

#include <stdio.h>
#include "trace.h"
#include "tools.h"

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------

/** Configuration for debug/logger port */
static const SerialConfig _SD2_CONFIG = {
   .speed = 1000000,
};

//-----------------------------------------------------------------------------
// Variables
//-----------------------------------------------------------------------------

/** Logger initialisation */
struct trace_port trace_logger = {
   .tp_ch = (BaseSequentialStream *)&SD2,
};

//-----------------------------------------------------------------------------
// API
//-----------------------------------------------------------------------------

void trace_init(void)
{
   chMtxObjectInit(&trace_logger.tp_mtx);

   // UART2: debug port
   sdStart(&SD2, &_SD2_CONFIG);
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
trace_hex(char * dst, size_t dlen, const void * buffer, size_t blen)
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
