cmake_minimum_required(VERSION 3.5)
project(vitis_common)

# NOTE, see the following resources to build a headers-only ROS 2 library
#  - https://docs.ros.org/en/foxy/Guides/Ament-CMake-Documentation.html#building-a-library
#  - https://github.com/gleichdick/ros2_header_only_lib
#  - https://dominikberner.ch/cmake-interface-lib/

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

#if (ROS_VITIS)
  find_package(OpenCL REQUIRED)

  ##################################
  # vitis_common header-only library
  ##################################
  add_library(${PROJECT_NAME}_header INTERFACE)
  target_include_directories(${PROJECT_NAME}_header INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  )
  target_link_libraries(${PROJECT_NAME}_header INTERFACE)


  ##################################
  # vitis_common library
  ##################################
  add_library(${PROJECT_NAME} SHARED
    src/utilities.cpp
  )
  target_include_directories(${PROJECT_NAME} PRIVATE
      include/
      # "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      # "$<INSTALL_INTERFACE:include>"
  )
  target_link_libraries(${PROJECT_NAME} ${OpenCL_LIBRARY})
  # ament_target_dependencies(${PROJECT_NAME}
  #   ${PROJECT_NAME}
  # )

  install(TARGETS ${PROJECT_NAME}_header ${PROJECT_NAME}
    EXPORT "export_${PROJECT_NAME}"
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib
    INCLUDES DESTINATION include
  )

  install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
  )

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_export_targets("export_${PROJECT_NAME}")
  # ament_export_dependencies(some_dependency)
#endif()  # ROS_VITIS

ament_package()
