/**
 * USB CDC communications
 */

#ifndef _USB_H_
#define _USB_H_

#include <stdint.h>

//-----------------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------------

#define comm_send_reply(_buf_, _size_) usb_send_reply(_buf_, _size_)
#define comm_debug_msg(_cat_, _val_) usb_debug_msg(_cat_, _val_)

//-----------------------------------------------------------------------------
// API
//-----------------------------------------------------------------------------

void usb_init(void);
void usb_start(void);
void usb_send_reply(const uint8_t * buffer, uint8_t size);
void usb_debug_msg(uint8_t category, uint16_t value);

#endif  /* _USB_H_ */
