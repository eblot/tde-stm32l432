#-----------------------------------------------------------------------------
# ChibiOS NIL kernel
#
# @file CMakeLists.txt
#-----------------------------------------------------------------------------

build_component_from (
  AUTO_INCLUDE
  ${CMAKE_SOURCE_DIR}/os/common/oslib/src/chmemcore.c
  ${CMAKE_SOURCE_DIR}/os/common/oslib/src/chmboxes.c
  ${CMAKE_SOURCE_DIR}/os/common/oslib/src/chmempools.c
  ${CMAKE_SOURCE_DIR}/os/common/oslib/src/chheap.c
  src/ch.c
)
