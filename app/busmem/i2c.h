/**
 * I2C slave API.
 */

#ifndef _I2C_H_
#define _I2C_H_

#include <stdint.h>

//-----------------------------------------------------------------------------
// Public API
//-----------------------------------------------------------------------------

void i2c_init(void);
void i2c_start(void);
void i2c_reset(bool irq_context);

#endif // _I2C_H_
