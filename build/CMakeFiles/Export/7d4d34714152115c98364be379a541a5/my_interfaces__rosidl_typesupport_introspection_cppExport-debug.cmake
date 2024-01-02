#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "my_interfaces::my_interfaces__rosidl_typesupport_introspection_cpp" for configuration "Debug"
set_property(TARGET my_interfaces::my_interfaces__rosidl_typesupport_introspection_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(my_interfaces::my_interfaces__rosidl_typesupport_introspection_cpp PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libmy_interfaces__rosidl_typesupport_introspection_cpp.so"
  IMPORTED_SONAME_DEBUG "libmy_interfaces__rosidl_typesupport_introspection_cpp.so"
  )

list(APPEND _cmake_import_check_targets my_interfaces::my_interfaces__rosidl_typesupport_introspection_cpp )
list(APPEND _cmake_import_check_files_for_my_interfaces::my_interfaces__rosidl_typesupport_introspection_cpp "${_IMPORT_PREFIX}/lib/libmy_interfaces__rosidl_typesupport_introspection_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
