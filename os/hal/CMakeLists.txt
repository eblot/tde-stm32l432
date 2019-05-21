#-----------------------------------------------------------------------------
# ChibiOS HAL
#
# @file CMakeLists.txt
#-----------------------------------------------------------------------------

INCLUDE_DIRECTORIES (include
                     lib/peripherals/displays
                     lib/peripherals/flash
                     lib/peripherals/sensors)

build_component_from (
  AUTO_INCLUDE
  boards/${BOARD}/board.c
  lib/streams/nullstreams.c
  lib/streams/chprintf.c
  lib/streams/memstreams.c
  ports/common/ARMCMx/nvic.c
  ports/STM32/STM32L4xx/hal_ext_lld_isr.c
  ports/STM32/STM32L4xx/hal_lld.c
  ports/STM32/LLD/DACv1/hal_dac_lld.c
  ports/STM32/LLD/DMAv1/stm32_dma.c
  ports/STM32/LLD/SPIv2/hal_i2s_lld.c
  ports/STM32/LLD/SPIv2/hal_spi_lld.c
  ports/STM32/LLD/RTCv2/hal_rtc_lld.c
  ports/STM32/LLD/USARTv2/hal_serial_lld.c
  ports/STM32/LLD/USARTv2/hal_uart_lld.c
  ports/STM32/LLD/I2Cv2/hal_i2c_lld.c
  ports/STM32/LLD/CANv1/hal_can_lld.c
  ports/STM32/LLD/ADCv3/hal_adc_lld.c
  ports/STM32/LLD/TIMv1/hal_pwm_lld.c
  ports/STM32/LLD/TIMv1/hal_st_lld.c
  ports/STM32/LLD/TIMv1/hal_gpt_lld.c
  ports/STM32/LLD/TIMv1/hal_icu_lld.c
  ports/STM32/LLD/USBv1/hal_usb_lld.c
  ports/STM32/LLD/SDMMCv1/hal_sdc_lld.c
  ports/STM32/LLD/MACv1/hal_mac_lld.c
  ports/STM32/LLD/GPIOv3/hal_pal_lld.c
  ports/STM32/LLD/QUADSPIv1/hal_qspi_lld.c
  ports/STM32/LLD/xWDGv1/hal_wdg_lld.c
  ports/STM32/LLD/EXTIv1/hal_ext_lld.c
  src/hal_mmcsd.c
  src/hal_pal.c
  src/hal_mmc_spi.c
  src/hal_ext.c
  src/hal_mac.c
  src/hal_qspi.c
  src/hal_i2c.c
  src/hal_queues.c
  src/hal_buffers.c
  src/hal_spi.c
  src/hal_usb.c
  src/hal_rtc.c
  src/hal_gpt.c
  src/hal_i2s.c
  src/hal_serial.c
  src/hal_pwm.c
  src/hal_uart.c
  src/hal.c
  src/hal_can.c
  src/hal_st.c
  src/hal_wdg.c
  src/hal_dac.c
  src/hal_icu.c
  src/hal_adc.c
  src/hal_sdc.c
  src/hal_serial_usb.c)
