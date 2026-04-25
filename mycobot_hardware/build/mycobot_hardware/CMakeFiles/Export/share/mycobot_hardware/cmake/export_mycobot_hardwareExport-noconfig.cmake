#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "mycobot_hardware::mycobot_hardware" for configuration ""
set_property(TARGET mycobot_hardware::mycobot_hardware APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(mycobot_hardware::mycobot_hardware PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libmycobot_hardware.so"
  IMPORTED_SONAME_NOCONFIG "libmycobot_hardware.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS mycobot_hardware::mycobot_hardware )
list(APPEND _IMPORT_CHECK_FILES_FOR_mycobot_hardware::mycobot_hardware "${_IMPORT_PREFIX}/lib/libmycobot_hardware.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
