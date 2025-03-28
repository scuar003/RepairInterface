cmake_minimum_required(VERSION 3.8)
project(demo_cpp_ready)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ur_rtde REQUIRED)

#UR_RTDE set up
# Manually specify the paths to ur_rtde
option(USE_DOCKER "Use Docker configuration" ON)

# Handles docker llibrary
if(USE_DOCKER)
  set(UR_RTDE_DIR "$ENV{WORKSPACE}/libs_/ur_rtde")
else()
  set(UR_RTDE_DIR "/home/robotics/p_devel/libs_/ur_rtde")
endif()

set(UR_RTDE_INCLUDE_DIR "${UR_RTDE_DIR}/include")
set(UR_RTDE_LIBRARY "${UR_RTDE_DIR}/build/librtde.so")

# Include directories
include_directories(
  ${UR_RTDE_INCLUDE_DIR}
)

# Debugging output to verify paths
message(STATUS "UR_RTDE_INCLUDE_DIR: ${UR_RTDE_INCLUDE_DIR}")
message(STATUS "UR_RTDE_LIBRARY: ${UR_RTDE_LIBRARY}")

# Add executables
add_executable(lidar_scan src/lidar_scan.cpp)
ament_target_dependencies(lidar_scan rclcpp std_msgs)
target_link_libraries(lidar_scan ${UR_RTDE_LIBRARY})



find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(main_menu src/main_menu.cpp)
ament_target_dependencies(main_menu rclcpp visualization_msgs interactive_markers geometry_msgs std_msgs)

add_executable(repair_interface src/repair_interface.cpp)
ament_target_dependencies(repair_interface rclcpp visualization_msgs interactive_markers geometry_msgs std_msgs)


# Install executable
install(TARGETS
  repair_interface  #Interface for visualization and task operation 
  main_menu         # Main menu of all available operations. 
  lidar_scan
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
