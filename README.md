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
$ https://github.com/Xilinx/kria_ros_perception.git
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

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
```
