cmake_minimum_required(VERSION 3.8)
project(point_cloud_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# ---------------------------------------------------------------------
# find dependencies
# ---------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(my_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.12 REQUIRED)
# ---------------------------------------------------------------------

# ---------------------------------------------------------------------
# point_cloud_processing
# ---------------------------------------------------------------------
add_executable(point_cloud_processing src/point_cloud_processing.cpp)

target_include_directories(point_cloud_processing PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(point_cloud_processing PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  point_cloud_processing
  rclcpp
  std_msgs
  sensor_msgs
  my_interfaces
  pcl_conversions
)
# ---------------------------------------------------------------------

# ---------------------------------------------------------------------
# include folder
# ---------------------------------------------------------------------
install(TARGETS point_cloud_processing
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})
  
target_link_libraries(point_cloud_processing
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# ---------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
