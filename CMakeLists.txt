cmake_minimum_required(VERSION 3.8)
project(mobil_robot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(megoldas src/megoldas.cpp)
ament_target_dependencies(megoldas rclcpp rcl_interfaces sensor_msgs geometry_msgs)

install(TARGETS megoldas DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()