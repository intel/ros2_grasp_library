find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grasp_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
set(TEST_NAME tgrasp_ros2)

ament_add_gtest(${TEST_NAME} tgrasp_ros2.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../src/consts.cpp)

if(TARGET ${TEST_NAME})
  get_filename_component(RESOURCE_DIR "resource" ABSOLUTE)
  configure_file(tgrasp_ros2.h.in tgrasp_ros2.h)
  include_directories(${CMAKE_CURRENT_BINARY_DIR} ${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  target_include_directories(${TEST_NAME} PUBLIC
    ${grasp_library_INCLUDE_DIRS}
  )
  ament_target_dependencies(${TEST_NAME}
    pcl_conversions
    rclcpp
    grasp_msgs
    moveit_msgs
    sensor_msgs
  )
  target_link_libraries(${TEST_NAME} ${GTEST_LIBRARIES} ${PCL_LIBRARIES})

  # Install binaries
  install(TARGETS ${TEST_NAME}
    RUNTIME DESTINATION bin
  )

  install(TARGETS ${TEST_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )

endif()
