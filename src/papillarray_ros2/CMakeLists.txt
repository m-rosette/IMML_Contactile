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

rosidl_generate_interfaces(${PROJECT_NAME}
  # "msg/PillarState.msg" #MIGHT NEED TO UNDO MY CHANGES TO THE VARIABLE NAMES
  # "msg/SensorState.msg"
  "srv/StartSlipDetection.srv"
  "srv/StopSlipDetection.srv"
  "srv/BiasRequest.srv"
)

include_directories(include)
add_executable(papillarray_ros_node src/papillarray_ros_node.cpp)
# include_directories(
#   include
#   lib
#   ${rclcpp_INCLUDE_DIRS}
# )
ament_target_dependencies(papillarray_ros_node
  rclcpp
  std_msgs)

install(TARGETS papillarray_ros_node
  DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(std_msgs)

# link_directories(
#   lib
# )

# include_directories(include/papillarray_ros2)

# add_executable(${PROJECT_NAME}_node 
# 	src/papillarray_ros_node.cpp
# )

# ament_target_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(${PROJECT_NAME}_node 
#   ${rclcpp_LIBRARIES}
# 	libPTSDK.a
# )

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
