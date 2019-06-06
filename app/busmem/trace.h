/**
 * Debug trace
 */

#ifndef _TRACE_H_
#define _TRACE_H_

#include <stdint.h>
#include <sys/types.h>
#include "hal.h"
#include "ch.h"
#include "chprintf.h"

//-----------------------------------------------------------------------------
// Type definitions
//-----------------------------------------------------------------------------

struct trace_port {
   BaseSequentialStream * tp_ch;
   mutex_t                tp_mtx;
};

//-----------------------------------------------------------------------------
// Type definitions
//-----------------------------------------------------------------------------

extern struct trace_port trace_logger;

//-----------------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------------

#define CR "\r"
#define LF "\n"
#define CRLF CR LF

#define MSGV(_fmt_, ...) \
   chMtxLock(&trace_logger.tp_mtx); \
   chprintf(trace_logger.tp_ch, _fmt_ CRLF, ##__VA_ARGS__); \
   chMtxUnlock(&trace_logger.tp_mtx);

#define MSGVN(_fmt_, ...) \
   chMtxLock(&trace_logger.tp_mtx); \
   chprintf(trace_logger.tp_ch, _fmt_, ##__VA_ARGS__); \
   chMtxUnlock(&trace_logger.tp_mtx);

//-----------------------------------------------------------------------------
// Public API
//-----------------------------------------------------------------------------

void trace_init(void);
size_t trace_hex(char * dst, size_t dlen, const void * buffer, size_t blen);

#endif // _TRACE_H_
