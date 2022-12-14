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
project(image_pipeline_examples)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_proc REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# LTTng flame chart and graphs
# see https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/LTTng-UST-Analyses.html#Flame_Chart_View
set(CMAKE_CXX_FLAGS "-g -O2 -finstrument-functions")

# Node built with a "component"
add_executable(rectify_node src/rectify_node.cpp)
ament_target_dependencies(rectify_node rclcpp class_loader image_proc)

add_executable(rectify_resize_node src/rectify_resize_node.cpp)
ament_target_dependencies(rectify_resize_node rclcpp class_loader image_proc)

  find_package(vitis_common REQUIRED)

  add_executable(rectify_resize_fpga_streamlined_node src/rectify_resize_fpga_streamlined_node.cpp)
  ament_target_dependencies(rectify_resize_fpga_streamlined_node rclcpp class_loader vitis_common image_proc)
  target_include_directories(rectify_resize_fpga_streamlined_node PUBLIC
    /home/xilinx/XRT/build/Debug/opt/xilinx/xrt/include  # TODO: replace with ROS 2 header-only library
    /usr/include/xrt  # TODO: replace with ROS 2 header-only library
  )

  if (DEFINED ROS_XRT)
    add_executable(rectify_resize_fpga_streamlined_node_xrt src/rectify_resize_fpga_streamlined_node_xrt.cpp)
    ament_target_dependencies(rectify_resize_fpga_streamlined_node_xrt rclcpp class_loader vitis_common image_proc)
    target_include_directories(rectify_resize_fpga_streamlined_node_xrt PUBLIC
      /home/xilinx/XRT/build/Debug/opt/xilinx/xrt/include  # TODO: replace with ROS 2 header-only library
      /usr/include/xrt  # TODO: replace with ROS 2 header-only library
    )
  endif()  # ROS_XRT

  #add_executable(rectify_resize_fpga_integrated_streamlined_node src/rectify_resize_fpga_integrated_streamlined_node.cpp)
  #ament_target_dependencies(rectify_resize_fpga_integrated_streamlined_node rclcpp class_loader vitis_common image_proc)
  #target_include_directories(rectify_resize_fpga_integrated_streamlined_node PUBLIC
  #  /home/xilinx/XRT/build/Debug/opt/xilinx/xrt/include  # TODO: replace with ROS 2 header-only library
  #  /usr/include/xrt  # TODO: replace with ROS 2 header-only library
  #)

# targets
set (install_targets
  rectify_node
  rectify_resize_node
)

set (install_targets ${install_targets}  
    rectify_resize_fpga_streamlined_node
 #   rectify_resize_fpga_integrated_streamlined_node
  )

install(
  TARGETS ${install_targets}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
