#-----------------------------------------------------------------------------
# Applicationz based on STM32L432
#
# @file CMakeLists.txt
#-----------------------------------------------------------------------------

# Import useful macros
CMAKE_MINIMUM_REQUIRED (VERSION 3.5)

PROJECT (stml4 C ASM)

use_target ()
use_default_xcc_warnings ()
use_default_xcc_settings ()
use_default_xas_settings ()

#-----------------------------------------------------------------------------
# CMake definitions
#-----------------------------------------------------------------------------

SET (DEVICE STM32L432xC)
SET (BOARD ST_NUCLEO32_L432KC)
SET (CHTYPE rt)

STRING (REGEX REPLACE "^(STM32L[0-9]).*$" "\\1xx" DEVICE_FAMILY ${DEVICE})

INCLUDE_DIRECTORIES (
  ${CMAKE_SOURCE_DIR}/config
  ${CMAKE_SOURCE_DIR}/os/license
  ${CMAKE_SOURCE_DIR}/os/common/ports/ARMCMx
  ${CMAKE_SOURCE_DIR}/os/common/ports/ARMCMx/compilers/GCC
  ${CMAKE_SOURCE_DIR}/os/common/startup/ARMCMx/devices/${DEVICE_FAMILY}
  ${CMAKE_SOURCE_DIR}/os/common/ext/ARM/CMSIS/Core/Include
  ${CMAKE_SOURCE_DIR}/os/common/ext/ST/${DEVICE_FAMILY}
  ${CMAKE_SOURCE_DIR}/os/hal/boards/${BOARD}
  ${CMAKE_SOURCE_DIR}/os/hal/include
  ${CMAKE_SOURCE_DIR}/os/hal/ports/common/ARMCMx
  ${CMAKE_SOURCE_DIR}/os/hal/ports/STM32/${DEVICE_FAMILY}
  ${CMAKE_SOURCE_DIR}/os/${CHTYPE}/include
  ${CMAKE_SOURCE_DIR}/os/hal/osal/${CHTYPE}
  ${CMAKE_SOURCE_DIR}/os/hal/ports/STM32/LLD/ADCv3
  ${CMAKE_SOURCE_DIR}/os/hal/ports/STM32/LLD/DACv1
  ${CMAKE_SOURCE_DIR}/os/hal/ports/STM32/LLD/DMAv1
  ${CMAKE_SOURCE_DIR}/os/hal/ports/STM32/LLD/EXTIv1
  ${CMAKE_SOURCE_DIR}/os/hal/ports/STM32/LLD/GPIOv3
  ${CMAKE_SOURCE_DIR}/os/hal/ports/STM32/LLD/I2Cv2
  ${CMAKE_SOURCE_DIR}/os/hal/ports/STM32/LLD/SPIv2
  ${CMAKE_SOURCE_DIR}/os/hal/ports/STM32/LLD/TIMv1
  ${CMAKE_SOURCE_DIR}/os/hal/ports/STM32/LLD/USARTv2
  ${CMAKE_SOURCE_DIR}/os/hal/ports/STM32/LLD/USBv1
  ${CMAKE_SOURCE_DIR}/os/hal/lib/streams
  ${CMAKE_SOURCE_DIR}/os/oslib/include)

ADD_DEFINITIONS (-Wno-documentation
                 -Wno-documentation-unknown-command)

ADD_DEFINITIONS (-DCRT1_AREAS_NUMBER=1 -D${BOARD})

find_subprojects (SUBPROJECTS)

STRING (REGEX REPLACE "^(.*L[0-9]+)[A-Z]([0-9]+)$" "\\1x\\2" brdtype ${BOARD})
SET (LINKSCRIPT ${DEVICE}.ld)
SET (LINKDIR ${CMAKE_SOURCE_DIR}/os/common/startup/ARMCMx/compilers/GCC/ld)

#-----------------------------------------------------------------------------
# Build configuration
#-----------------------------------------------------------------------------

# build all projects
FOREACH (project ${SUBPROJECTS})
  ADD_SUBDIRECTORY (${project} ${project})
ENDFOREACH ()
