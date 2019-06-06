/**
 * Dummy memory storage
 *    for STM32L432KC (Nucleo Board)
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "hal.h"
#include "ch.h"
#include "cmd.h"
#include "i2c.h"
#include "trace.h"
#include "usb.h"

//-----------------------------------------------------------------------------
// Physical pad usage (STM32L432KC Nucleo-32)
//-----------------------------------------------------------------------------
// Nucleo-32 PCB
//   Beware that ST PCBs are shipped with SB16 & SB18 which should be
//   removed or PA6/PB6 and PA5/PB7 are short-circuited.
//
// I2C slave
//   PA9/D1: I2C1 SCL
//   PA10/D0: I2C1 SDA
//
// SPI slave
//   PA7/A6: SPI1 MOSI
//   PA6/A5: SPI1 MISO
//   PA5/A4: SPI1 SCLK
//   PA4/A3: SPI1 /CS
//
// Host USB I/F:
//   PA11/DM: USB D-
//   PA12/DP: USB D+
//
// Host debug:
//   USB (STM32F1) for SWD access and VCP UART TX debug (@ 1Mbps)
//   PA2/A7: UART2 TX
//-----------------------------------------------------------------------------


int
main(void)
{
   halInit();
   chSysInit();
   trace_init();

   //cmd_init();
   usb_init();
   i2c_init();

   usb_start();
   i2c_start();
   for(unsigned int loop=0; loop<UINT32_MAX/2; loop++) {
      chThdSleepMilliseconds(10);
   }

   return 0;
}
