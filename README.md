# kria_ros_perception
Kria pre-built ROS perception example app

## Gazebo simulation setup on workstation running Ubuntu 22.04

### Installing ROS Humble and gazebo classic
- Follow [these](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) instructions to install ros on x86 machine
- Install Gazebo Classic from [here](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)
> :warning: ROS 2 Humble ships with Ignition Gazebo (renamed to “Gazebo”). Installing Gazebo Classic (Gazebo 11.0) is still possible but requires some manual work. Some of the examples below were developed with Gazebo Classic. In turn, examples might be rewritten with Ignition Gazebo (“Gazebo”) to facilitate the flows.

### Environment Setup On Host Machine(Ubuntu 22.04)

1. Install ROS2 using debian package, see install guide [here](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html).
2. Install few more dependencies from ros-humble using the below command
```bash
$ sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-plugins ros-humble-gazebo-msgs python3-colcon-common-extensions
```
3. Install Gazebo Classic 11.0, see install guide [here](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)
4. Get files to setup and run simulation
```bash
$ git clone https://github.com/Xilinx/kria_ros_perception
$ cd kria_ros_perception
$ rm -rf src/image_proc src/tracetools_image_pipeline src/vitis_common src/tracing src/image_pipeline_examples
```
3. Install Simulation from the below steps
```bash
$ source /opt/ros/humble/setup.bash
$ colcon build
```
4. Run simulation on workstation
```bash
$ source install/setup.bash  # source the workspace as an overlay
$ ros2 launch perception_2nodes simulation.launch.py
```

# Running perception on KRIA

## Install ROS Humble
- Follow [these](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) instructions to install ros on KRIA kr260

## Install Perception application
- Install the application debian using the below command. It will install the app along with all the dependencies required for running the the perception node
```
sudo apt install -y ros-humble-xlnx-ros-perception_0.1.0-0jammy_arm64.deb
```
> :warning: User is expected to install ROS Humble using the above mentioned link. Otherwise the application may not work


## Running CPU only varient
```
ros2 launch perception_2nodes trace_rectify_resize.launch.py
```

## Running Streamlining Accelerated varient

  - This Using AXI4-Stream interface to create an intra-FPGA ROS 2 communication queue which is used to pass data across nodes through the FPGA
  ```
  xmutil unloadapp
  xmutil loadapp image_proc_streamlined
  ros2 launch perception_2nodes trace_rectify_resize_fpga_streamlined.launch.py
  ```
## License

```
Copyright (C) 2022, Advanced Micro Devices, Inc.  All rights reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```
