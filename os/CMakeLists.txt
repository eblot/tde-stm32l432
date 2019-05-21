#-----------------------------------------------------------------------------
# ChibiOS libraries
#
# @file CMakeLists.txt
#-----------------------------------------------------------------------------

# define subprojects
SET (subprojects
     common
     hal
     rt
     various)

#-----------------------------------------------------------------------------
# Build configuration
#-----------------------------------------------------------------------------

ADD_DEFINITIONS (-Wno-missing-prototypes
                 -Wno-conversion
                 -Wno-sign-conversion
                 -Wno-bad-function-cast
                 -Wno-missing-noreturn
                 -Wno-cast-align
                 -Wno-switch-enum
                 -Wno-unreachable-code-break)

# build all projects
FOREACH (project ${subprojects})
  ADD_SUBDIRECTORY (${project} ${project})
ENDFOREACH ()
