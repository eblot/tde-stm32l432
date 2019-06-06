/**
 * Command manager.
 */

#ifndef _CMD_H_
#define _CMD_H_

#include <stdint.h>
#include <stddef.h>

/** Maximal command size */
#define CMD_FRAME_SIZE   64U

void cmd_init(void);
void cmd_reset(void);
void cmd_receive_byte(uint8_t byte);
void cmd_stream_msg(const uint8_t * buf, size_t length);
void cmd_signal_eos(void);
void cmd_signal_usb(bool available);

#endif // _CMD_H_
