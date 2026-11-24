cmake_minimum_required(VERSION 3.16)
project(ekf2d_ros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# The filter itself lives in the standalone project two folders up; compile
# those sources here so there is only one copy of the EKF and UTM code.
set(EKF_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

add_executable(ekf2d_node
  src/ekf2d_node.cpp
  ${EKF_ROOT}/src/ekf2d.cpp
  ${EKF_ROOT}/src/utm.cpp
)
target_include_directories(ekf2d_node PRIVATE ${EKF_ROOT}/include)
target_link_libraries(ekf2d_node Eigen3::Eigen)
ament_target_dependencies(ekf2d_node
  rclcpp sensor_msgs geometry_msgs nav_msgs visualization_msgs tf2_ros)

install(TARGETS ekf2d_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
