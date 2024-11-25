cmake_minimum_required(VERSION 3.8)

# Define the project name
project(walker)

# Add compiler options for warnings when using GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic) # Enable common warning flags
endif()

# Generate compile_commands.json for development tools like clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")

# Find required dependencies
find_package(ament_cmake REQUIRED)     # Core ROS2 build tool
find_package(rclcpp REQUIRED)          # ROS2 C++ client library
find_package(geometry_msgs REQUIRED)   # Geometry messages for Twist
find_package(std_msgs REQUIRED)        # Standard messages
find_package(sensor_msgs REQUIRED)     # Sensor messages for LaserScan

# Specify the directories for include files
include_directories(include)

# Add an executable for the Walker node
add_executable(walker src/walker.cpp)

# Link required dependencies to the executable
ament_target_dependencies(walker rclcpp geometry_msgs std_msgs sensor_msgs)

# Install the executable to the appropriate location
install(TARGETS
  walker
  DESTINATION lib/${PROJECT_NAME}) # ROS2 convention for installing executables

# Install header files for external use
install(DIRECTORY
  include/
  DESTINATION include)

# Install the launch files for ROS2 to locate and run them
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Enable testing if the BUILD_TESTING flag is set
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED) # Automatically configure linting tools
  
  # Skip copyright linter if the package does not yet include a license
  set(ament_cmake_copyright_FOUND TRUE)
  
  # Skip cpplint if this package is not in a Git repository
  set(ament_cmake_cpplint_FOUND TRUE)
  
  # Find test dependencies required for linting and testing
  ament_lint_auto_find_test_dependencies()
endif()

# Mark this as a ROS2 package
ament_package()

