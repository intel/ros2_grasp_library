find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grasp_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

ament_add_gtest(tgrasp_library tgrasp_library.cpp)
if(TARGET tgrasp_library)
  target_include_directories(tgrasp_library PUBLIC
    ${grasp_library_INCLUDE_DIRS}
  )
  ament_target_dependencies(tgrasp_library
    rclcpp
    grasp_msgs
    moveit_msgs
  )
  target_link_libraries(tgrasp_library ${GTEST_LIBRARIES})
endif()
