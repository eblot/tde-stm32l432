#-----------------------------------------------------------------------------
# ChibiOS Real-Time kernel
#
# @file CMakeLists.txt
#-----------------------------------------------------------------------------

INCLUDE_DIRECTORIES (${CMAKE_SOURCE_DIR}/os/common/oslib/include
                     os/common/oslib/)

build_component_from (
  AUTO_INCLUDE
  src/chsys.c
  src/chdebug.c
  src/chtrace.c
  src/chvt.c
  src/chschd.c
  src/chthreads.c
  src/chtm.c
  src/chstats.c
  src/chregistry.c
  src/chsem.c
  src/chmtx.c
  src/chcond.c
  src/chevents.c
  src/chmsg.c
  src/chdynamic.c
  ${CMAKE_SOURCE_DIR}/os/common/oslib/src/chmboxes.c
  ${CMAKE_SOURCE_DIR}/os/common/oslib/src/chmemcore.c
  ${CMAKE_SOURCE_DIR}/os/common/oslib/src/chheap.c
  ${CMAKE_SOURCE_DIR}/os/common/oslib/src/chmempools.c)
