#-----------------------------------------------------------------------------
# Common library for ChibiOS
#
# @file CMakeLists.txt
#-----------------------------------------------------------------------------


INCLUDE_DIRECTORIES (abstractions/nasa_cfe/osal/include
                     abstractions/nasa_cfe/psp/include
                     ext/CMSIS/ST/${DEVICE_FAMILY}
                     ext/CMSIS/generic
                     ext/CMSIS/include
                     oslib/include
                     startup/ARMCMx/devices/${DEVICE_FAMILY})

ADD_DEFINITIONS (-Wno-inline-asm)

build_component_from (
  AUTO_INCLUDE
  abstractions/cmsis_os/cmsis_os.c
  startup/ARMCMx/compilers/GCC/crt0_v7m.S
  startup/ARMCMx/compilers/GCC/crt1.c
  startup/ARMCMx/compilers/GCC/vectors.S
  ports/ARMCMx/compilers/GCC/chcoreasm_v7m.S
  ports/ARMCMx/chcore_v7m.c
  ports/ARMCMx/chcore.c
)
