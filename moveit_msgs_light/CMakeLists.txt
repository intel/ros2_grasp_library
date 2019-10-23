cmake_minimum_required(VERSION 3.5)

project(moveit_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(msg_files
  "msg/CollisionObject.msg"
  "msg/Grasp.msg"
  "msg/GripperTranslation.msg"
  "msg/MoveItErrorCodes.msg"
  "msg/ObjectType.msg"
  "msg/PlaceLocation.msg"
)

set(srv_files
  "srv/GraspPlanning.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs shape_msgs trajectory_msgs
  ADD_LINTER_TESTS
)

# todo install ros1_bridge mapping rules

ament_export_dependencies(rosidl_default_runtime)

ament_package()
