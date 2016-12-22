#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "cozmo" for configuration ""
set_property(TARGET cozmo APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(cozmo PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libcozmo.so"
  IMPORTED_SONAME_NOCONFIG "libcozmo.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS cozmo )
list(APPEND _IMPORT_CHECK_FILES_FOR_cozmo "${_IMPORT_PREFIX}/lib/libcozmo.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
