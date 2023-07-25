/*
      ____  ____
     /   /\/   /
    /___/  \  /   Copyright (c) 2021, Xilinx®.
    \   \   \/    Author: Víctor Mayoral Vilches <victorma@xilinx.com>
     \   \
     /   /        Licensed under the Apache License, Version 2.0 (the "License");
    /___/   /\    you may not use this file except in compliance with the License.
    \   \  /  \   You may obtain a copy of the License at
     \___\/\___\            http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

    Inspired by rectify.hpp authored by Willow Garage, Inc., Andreas Klintberg,
      Joshua Whitley
*/

#ifndef IMAGE_PROC__RECTIFY_RESIZE_FPGA_HPP_
#define IMAGE_PROC__RECTIFY_RESIZE_FPGA_HPP_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <thread>
#include <memory>
#include <vector>
#include <string>

#include <vitis_common/common/ros_opencl_120.hpp>

namespace image_geometry
{

  class PinholeCameraModelFPGAStreamlined : public image_geometry::PinholeCameraModel
  {
  public:
    /* \brief Constructor
     */
    PinholeCameraModelFPGAStreamlined();

    /**
     * \brief Rectify a raw camera image offloading the remapping to the FPGA.
     *
     *  TODO: Consider pushing OpenCV cv::initRectificationMaps also to the FPGA
     *  by using Vitis Vision Library xf::cv::InitUndistortRectifyMapInverse
     */
    void rectifyresizeImageFPGA() const;

  private:
  };

} // namespace image_geometry

namespace image_proc
{

  class RectifyResizeNodeFPGAStreamlined
      : public rclcpp::Node
  {
  public:
    explicit RectifyResizeNodeFPGAStreamlined(const rclcpp::NodeOptions &);

  private:
    image_transport::CameraSubscriber sub_camera_;

    int queue_size_;
    int interpolation;

    bool use_scale_;
    bool profile_;
    double scale_height_;
    double scale_width_;
    int height_;
    int width_;
    bool custom_maps;

// ROS Input/Output images
    cv_bridge::CvImage output_image;
    sensor_msgs::msg::CameraInfo::SharedPtr dst_info_msg;
    cv_bridge::CvImagePtr cv_ptr;
    bool zero_distortion = true;
    bool gray;

// OpenCL
    cl_int err;
    cl::Kernel *krnl_rectify;
    cl::Kernel *krnl_resize;
    cl::Context *context_;
    cl::CommandQueue *queue_;
    cv::Mat custom_map_x, custom_map_y;
    cv::Mat hls_remapped;
    size_t image_in_size_bytes, map_in_size_bytes, image_out_size_bytes;
    uint8_t *ptr_a;
    uint32_t *ptr_x, *ptr_y;
    uint8_t *ptr_result;
    cl::Buffer buffer_inImage, buffer_inMapX, buffer_inMapY, buffer_outImage;

    std::mutex connect_mutex_;
    image_transport::Publisher pub_rect_;
    image_transport::CameraPublisher pub_image_;

    // Processing state (note: only safe because we're using single-threaded NodeHandle!)
    image_geometry::PinholeCameraModelFPGAStreamlined model_;

    void init();
    void subscribeToCamera();
    void imageCb(
        const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg);
  };

} // namespace image_proc

#endif // IMAGE_PROC__RECTIFY_RESIZE_FPGA_STREAMLINED_HPP_
