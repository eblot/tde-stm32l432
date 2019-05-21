#-----------------------------------------------------------------------------
# # ChibiOS miscelleanous libraries
#
# @file CMakeLists.txt
#-----------------------------------------------------------------------------

build_component_from (
  AUTO_INCLUDE
  evtimer.c
  shell/shell.c
  shell/shell_cmd.c)
