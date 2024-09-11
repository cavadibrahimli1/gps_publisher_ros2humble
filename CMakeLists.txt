cmake_minimum_required(VERSION 3.8)
project(ros_pospac_bridge)

# Add the custom CMake module path (if needed)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Check if the compiler is GCC or Clang and add compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(autoware_sensing_msgs REQUIRED)  # Add this line

ament_auto_find_build_dependencies()

# Include directories (for header files, if any)
include_directories(include)

# Add the executable
add_executable(ros_pospac_bridge src/ros_pospac_bridge.cpp)

# Link dependencies
ament_target_dependencies(ros_pospac_bridge 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  tf2 
  tf2_geometry_msgs 
  GeographicLib
  autoware_sensing_msgs  # Add this line
)

# Install the executable
install(TARGETS
  ros_pospac_bridge
  DESTINATION lib/${PROJECT_NAME})

# Install the launch directory (including launch files)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install the config directory (including YAML files)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
        launch
        config
)
