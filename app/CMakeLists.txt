#-----------------------------------------------------------------------------
# STML432 Applications
#
# @file CMakeLists.txt
#-----------------------------------------------------------------------------

# define subprojects
SET (subprojects)
LIST (APPEND subprojects
      usb-cdc)

#-----------------------------------------------------------------------------
# Build configuration
#-----------------------------------------------------------------------------

SET (symbols)
# main stack: exception stack
LIST (APPEND symbols LINK_SYMBOL __main_stack_size__=0x400)
# process stack: app thread stack
LIST (APPEND symbols LINK_SYMBOL __process_stack_size__=0x400)

# build all projects
FOREACH (project ${subprojects})
  ADD_SUBDIRECTORY (${project} ${project})
ENDFOREACH ()
