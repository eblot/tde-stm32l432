#-----------------------------------------------------------------------------
# STML432 Applications
#
# @file CMakeLists.txt
#-----------------------------------------------------------------------------

# define subprojects
find_subprojects (SUBPROJECTS)

#-----------------------------------------------------------------------------
# Build configuration
#-----------------------------------------------------------------------------

SET (symbols)
# main stack: exception stack
LIST (APPEND symbols LINK_SYMBOL __main_stack_size__=0x400)
# process stack: app thread stack
LIST (APPEND symbols LINK_SYMBOL __process_stack_size__=0x400)

# build all projects
FOREACH (project ${SUBPROJECTS})
  ADD_SUBDIRECTORY (${project} ${project})
ENDFOREACH ()
