#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Poco::Net" for configuration "None"
set_property(TARGET Poco::Net APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(Poco::Net PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/arm-linux-gnueabihf/libPocoNet.so.62"
  IMPORTED_SONAME_NONE "libPocoNet.so.62"
  )

list(APPEND _IMPORT_CHECK_TARGETS Poco::Net )
list(APPEND _IMPORT_CHECK_FILES_FOR_Poco::Net "${_IMPORT_PREFIX}/lib/arm-linux-gnueabihf/libPocoNet.so.62" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
