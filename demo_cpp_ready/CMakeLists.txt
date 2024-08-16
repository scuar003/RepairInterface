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
find_package( sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(tf2_eigen REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})

add_executable(main_menu src/main_menu.cpp)
ament_target_dependencies(main_menu rclcpp visualization_msgs interactive_markers geometry_msgs std_msgs)


add_executable(surface_detection src/surface_detection.cpp)
ament_target_dependencies(surface_detection rclcpp geometry_msgs std_msgs sensor_msgs pcl_conversions tf2_ros tf2_geometry_msgs)

target_link_libraries(surface_detection
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

add_executable(repair_interface src/repair_interface.cpp)
ament_target_dependencies(repair_interface rclcpp visualization_msgs interactive_markers geometry_msgs std_msgs tf2_ros tf2_eigen tf2_geometry_msgs)

# Install executable
install(TARGETS
  repair_interface  #Interface for visualization and task operation 
  surface_detection
  main_menu         # Main menu of all available operations. 
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
