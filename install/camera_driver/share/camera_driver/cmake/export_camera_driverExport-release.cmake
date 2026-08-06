#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "camera_driver::camera_imu_stabilizer" for configuration "Release"
set_property(TARGET camera_driver::camera_imu_stabilizer APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(camera_driver::camera_imu_stabilizer PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcamera_imu_stabilizer.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS camera_driver::camera_imu_stabilizer )
list(APPEND _IMPORT_CHECK_FILES_FOR_camera_driver::camera_imu_stabilizer "${_IMPORT_PREFIX}/lib/libcamera_imu_stabilizer.a" )

# Import target "camera_driver::camera_driver_component" for configuration "Release"
set_property(TARGET camera_driver::camera_driver_component APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(camera_driver::camera_driver_component PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcamera_driver_component.so"
  IMPORTED_SONAME_RELEASE "libcamera_driver_component.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS camera_driver::camera_driver_component )
list(APPEND _IMPORT_CHECK_FILES_FOR_camera_driver::camera_driver_component "${_IMPORT_PREFIX}/lib/libcamera_driver_component.so" )

# Import target "camera_driver::camera_driver_node" for configuration "Release"
set_property(TARGET camera_driver::camera_driver_node APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(camera_driver::camera_driver_node PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/camera_driver/camera_driver_node"
  )

list(APPEND _IMPORT_CHECK_TARGETS camera_driver::camera_driver_node )
list(APPEND _IMPORT_CHECK_FILES_FOR_camera_driver::camera_driver_node "${_IMPORT_PREFIX}/lib/camera_driver/camera_driver_node" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
