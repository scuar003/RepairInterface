cmake_minimum_required(VERSION 3.8)
project(repair_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Source files
set(SOURCES
  src/ros2_menu.cpp
)

# Executable and target dependencies
add_executable(main_menu src/main_menu.cpp ${SOURCES})
ament_target_dependencies(main_menu 
  rclcpp 
  visualization_msgs 
  interactive_markers 
  std_msgs
)

# Ensure include/repair_interface is part of the path
target_include_directories(main_menu PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Install executable
install(TARGETS
  main_menu
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory for headers
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
