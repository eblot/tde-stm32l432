#-----------------------------------------------------------------------------
# ChibiOS NIL kernel
#
# @file CMakeLists.txt
#-----------------------------------------------------------------------------

build_component_from (
  AUTO_INCLUDE
  ${CMAKE_SOURCE_DIR}/os/oslib/src/chfactory.c
  ${CMAKE_SOURCE_DIR}/os/oslib/src/chmboxes.c
  ${CMAKE_SOURCE_DIR}/os/oslib/src/chmemcore.c
  ${CMAKE_SOURCE_DIR}/os/oslib/src/chmemheaps.c
  ${CMAKE_SOURCE_DIR}/os/oslib/src/chmempools.c
  ${CMAKE_SOURCE_DIR}/os/oslib/src/chpipes.c
  src/ch.c
)
