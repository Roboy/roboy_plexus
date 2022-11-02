if (console_bridge_CONFIG_INCLUDED)
  return()
endif()
set(console_bridge_CONFIG_INCLUDED TRUE)


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was console_bridge-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../../" ABSOLUTE)

# Use original install prefix when loaded through a "/usr move"
# cross-prefix symbolic link such as /lib -> /usr/lib.
get_filename_component(_realCurr "${CMAKE_CURRENT_LIST_DIR}" REALPATH)
get_filename_component(_realOrig "/usr/lib/arm-linux-gnueabihf/console_bridge/cmake/" REALPATH)
if(_realCurr STREQUAL _realOrig)
  set(PACKAGE_PREFIX_DIR "/usr")
endif()
unset(_realOrig)
unset(_realCurr)

####################################################################################

set(console_bridge_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include")

include("${CMAKE_CURRENT_LIST_DIR}/console_bridge-targets.cmake")
set(console_bridge_LIBRARIES console_bridge::console_bridge)
