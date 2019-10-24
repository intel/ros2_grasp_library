cmake_minimum_required(VERSION 3.5)

project(handeye_tf_service)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(srv_files
  "srv/HandeyeTF.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES geometry_msgs builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(handeye_tf_server 
  src/handeye_tf_server.cpp
)

ament_target_dependencies(handeye_tf_server
  rclcpp tf2_ros
)

get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)
get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

foreach(typesupport_impl ${typesupport_impls})
  rosidl_target_interfaces(handeye_tf_server
    ${PROJECT_NAME} ${typesupport_impl}
  )
endforeach()

install(TARGETS handeye_tf_server
    DESTINATION lib/${PROJECT_NAME})

ament_package()