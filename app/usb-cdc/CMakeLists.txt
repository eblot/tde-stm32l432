#-----------------------------------------------------------------------------
# USB-CDC bridge
#
#-----------------------------------------------------------------------------

GET_FILENAME_COMPONENT (COMPONENT ${CMAKE_CURRENT_SOURCE_DIR} NAME)

use_newlib ()

tag_application (${COMPONENT} ${CMAKE_SOURCE_DIR})

INCLUDE_DIRECTORIES(include
                    ${CMAKE_CURRENT_BINARY_DIR}
                    ${CMAKE_SOURCE_DIR}/os/various/shell/)

ADD_EXECUTABLE (${COMPONENT}
                main.c
                usbcfg.c
                ${CMAKE_CURRENT_BINARY_DIR}/${TAGFILE_SRC})
ADD_DEFINITIONS (-DAPP_NAME=${COMPONENT})
ADD_FILE_DEPENDENCIES (main.c
                       ${CMAKE_CURRENT_BINARY_DIR}/${TAGFILE_HEADER})

link_app (${COMPONENT}
          common
          hal
          rt
          various
          ${symbols}
          LINK_SCRIPT ${LINKDIR}/${LINKSCRIPT}
          LINK_SCRIPT_DIR ${LINKDIR})

post_gen_app (${COMPONENT} ASM BIN SREC SIZE)
