#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Poco::Crypto" for configuration "None"
set_property(TARGET Poco::Crypto APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(Poco::Crypto PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/arm-linux-gnueabihf/libPocoCrypto.so.62"
  IMPORTED_SONAME_NONE "libPocoCrypto.so.62"
  )

list(APPEND _IMPORT_CHECK_TARGETS Poco::Crypto )
list(APPEND _IMPORT_CHECK_FILES_FOR_Poco::Crypto "${_IMPORT_PREFIX}/lib/arm-linux-gnueabihf/libPocoCrypto.so.62" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
