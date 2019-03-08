find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grasp_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(grasp_library REQUIRED)

ament_add_gtest(tgrasp_library tgrasp_library.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../src/consts.cpp)

if(TARGET tgrasp_library)
  get_filename_component(RESOURCE_DIR "resource" ABSOLUTE)
  configure_file(tgrasp_library.h.in tgrasp_library.h)
  include_directories(${CMAKE_CURRENT_BINARY_DIR} ${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  target_include_directories(tgrasp_library PUBLIC
    ${grasp_library_INCLUDE_DIRS}
  )
  ament_target_dependencies(tgrasp_library
    pcl_conversion
    rclcpp
    grasp_msgs
    moveit_msgs
    sensor_msgs
  )
  target_link_libraries(tgrasp_library ${GTEST_LIBRARIES} ${PCL_LIBRARIES})

  # Install binaries
  install(TARGETS tgrasp_library
    RUNTIME DESTINATION bin
  )

  install(TARGETS tgrasp_library
    DESTINATION lib/${PROJECT_NAME}
  )

endif()
