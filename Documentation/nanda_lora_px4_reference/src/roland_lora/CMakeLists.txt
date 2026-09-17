cmake_minimum_required(VERSION 3.8)
project(roland_lora)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(roland_lora_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bbox.msg"
  "msg/Bboxes.msg"
)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_executable(autoland_node src/autoland_node.cpp)
ament_target_dependencies(autoland_node rclcpp geometry_msgs mavros_msgs)
install(TARGETS autoland_node DESTINATION lib/${PROJECT_NAME})

add_executable(ekf_node src/ekf_node.cpp)
ament_target_dependencies(ekf_node rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs tf2_ros tf2_geometry_msgs cv_bridge OpenCV roland_lora_msgs)
rosidl_target_interfaces(ekf_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS ekf_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()

install(PROGRAMS
  scripts/vision_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/lora_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
