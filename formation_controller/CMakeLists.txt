cmake_minimum_required(VERSION 3.8)
project(formation_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------
# find dependencies
# ---------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# ---------------------------------------------------------------------

# ---------------------------------------------------------------------
# include folder
# ---------------------------------------------------------------------
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})
# ---------------------------------------------------------------------

# ---------------------------------------------------------------------
# desired_formation
# ---------------------------------------------------------------------
add_executable(desired_formation src/desired_formation.cpp)
target_include_directories(desired_formation PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(desired_formation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  desired_formation
  rclcpp
  geometry_msgs
  my_interfaces
)
  
install(TARGETS desired_formation
DESTINATION lib/${PROJECT_NAME})
# ---------------------------------------------------------------------

# ---------------------------------------------------------------------
# follower1_controller
# ---------------------------------------------------------------------
add_executable(follower1_controller src/follower1_controller.cpp)
target_include_directories(follower1_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(follower1_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  follower1_controller
  rclcpp
  geometry_msgs
  my_interfaces
)

install(TARGETS follower1_controller
DESTINATION lib/${PROJECT_NAME})
# ---------------------------------------------------------------------


# ---------------------------------------------------------------------
# follower2_controller
# ---------------------------------------------------------------------
add_executable(follower2_controller src/follower2_controller.cpp)
target_include_directories(follower2_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(follower2_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  follower2_controller
  rclcpp
  geometry_msgs
  my_interfaces
  nav_msgs
  tf2_geometry_msgs
)

install(TARGETS follower2_controller
DESTINATION lib/${PROJECT_NAME})
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
