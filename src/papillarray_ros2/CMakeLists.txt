cmake_minimum_required(VERSION 3.8)
project(papillarray_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include/${PROJECT_NAME}
  include
  lib)

add_executable(${PROJECT_NAME}_node 
  src/papillarray_ros2_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp
  std_msgs
)

install(TARGETS ${PROJECT_NAME}_node 
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Dont know if I need to include this...
# target_link_libraries(${PROJECT_NAME}_node
#   ${rclcpp_LIBRARIES}
#   libPTSDK.a
# )

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PillarState.msg"
  "msg/SensorState.msg"
  "srv/StartSlipDetection.srv"
  "srv/StopSlipDetection.srv"
  "srv/BiasRequest.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

rosidl_target_interfaces(${PROJECT_NAME}_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_include_directories(include)
ament_export_dependencies(std_msgs)
ament_package()
