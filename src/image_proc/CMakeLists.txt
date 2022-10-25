#
# Copyright (C) 2022, Advanced Micro Devices, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
cmake_minimum_required(VERSION 3.5)
project(image_proc)

# ROS2 Flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# LTTng flame chart and graphs
# see https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/LTTng-UST-Analyses.html#Flame_Chart_View
set(CMAKE_CXX_FLAGS "-g -O2 -finstrument-functions")

find_package(OpenCL REQUIRED)
find_package(vitis_common)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
if(OpenCV_VERSION VERSION_LESS "3.2.0")
  message(FATAL "Minimum OpenCV version is 3.2.0 (found version ${OpenCV_VERSION})")
endif()

include_directories(include)

# image_proc library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/processor.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

# rectify library
ament_auto_add_library(rectify SHARED
  src/rectify.cpp)
ament_target_dependencies(rectify tracetools_image_pipeline)
target_compile_definitions(rectify
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_nodes(rectify "image_proc::RectifyNode")
set(node_plugins "${node_plugins}image_proc::RectifyNode;$<TARGET_FILE:rectify>\n")

# resize library
ament_auto_add_library(resize SHARED
  src/resize.cpp
)
ament_target_dependencies(resize tracetools_image_pipeline)
target_compile_definitions(resize
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_nodes(resize "image_proc::ResizeNode")
set(node_plugins "${node_plugins}image_proc::ResizeNode;$<TARGET_FILE:resize>\n")

  ##########
  ## Streamlined components
  #
  ##########

  # rectify_resize_fpga_streamlined component - 1 Component, 2 kernels
#  ament_auto_add_library(rectify_resize_fpga_streamlined SHARED src/rectify_resize_fpga_streamlined.cpp)
#  target_include_directories(rectify_resize_fpga_streamlined PUBLIC
    # $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/vitis_common>
#  )
#  target_link_libraries(rectify_resize_fpga_streamlined ${OpenCL_LIBRARY})
#  ament_target_dependencies(rectify_resize_fpga_streamlined)
#  target_compile_definitions(rectify_resize_fpga_streamlined PRIVATE "COMPOSITION_BUILDING_DLL")
#  rclcpp_components_register_nodes(rectify_resize_fpga_streamlined "image_proc::RectifyResizeNodeFPGAStreamlined")
#  set(node_plugins "${node_plugins}image_proc::RectifyResizeNodeFPGAStreamlined;$<TARGET_FILE:rectify_resize_fpga_streamlined>\n")

  # rectify_fpga_streamlined component - 1 Kernel, 1 Component (part of group)
  ament_auto_add_library(rectify_fpga_streamlined SHARED src/rectify_fpga_streamlined.cpp)
  target_include_directories(rectify_fpga_streamlined PUBLIC
    # $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/vitis_common>
  )
  target_link_libraries(rectify_fpga_streamlined ${OpenCL_LIBRARY})
  target_compile_definitions(rectify_fpga_streamlined PRIVATE "COMPOSITION_BUILDING_DLL")
  rclcpp_components_register_nodes(rectify_fpga_streamlined "image_proc::RectifyNodeFPGAStreamlined")
  set(node_plugins "${node_plugins}image_proc::RectifyNodeFPGAStreamlined;$<TARGET_FILE:rectify_fpga_streamlined>\n")

  # resize_fpga_streamlined component - 1 Kernel, 1 Component (part of group)
  ament_auto_add_library(resize_fpga_streamlined SHARED src/resize_fpga_streamlined.cpp)
  target_include_directories(resize_fpga_streamlined PUBLIC
    # $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/vitis_common>
  )
  target_link_libraries(resize_fpga_streamlined ${OpenCL_LIBRARY})
  target_compile_definitions(resize_fpga_streamlined PRIVATE "COMPOSITION_BUILDING_DLL")
  rclcpp_components_register_nodes(resize_fpga_streamlined "image_proc::ResizeNodeFPGAStreamlined")
  set(node_plugins "${node_plugins}image_proc::ResizeNodeFPGAStreamlined;$<TARGET_FILE:resize_fpga_streamlined>\n")

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_libraries(image_proc "stdc++fs")
endif()

install(
  TARGETS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
