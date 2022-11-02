#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Poco::Data" for configuration "None"
set_property(TARGET Poco::Data APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(Poco::Data PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/arm-linux-gnueabihf/libPocoData.so.62"
  IMPORTED_SONAME_NONE "libPocoData.so.62"
  )

list(APPEND _IMPORT_CHECK_TARGETS Poco::Data )
list(APPEND _IMPORT_CHECK_FILES_FOR_Poco::Data "${_IMPORT_PREFIX}/lib/arm-linux-gnueabihf/libPocoData.so.62" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
