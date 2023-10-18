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

    Inspired by rectify.cpp authored by Willow Garage, Inc., Andreas Klintberg,
      Joshua Whitley. Inspired also by PinholeCameraModel class.
*/

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.hpp>
#include <vitis_common/common/utilities.hpp>
#include "opencv2/core.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <thread>
#include <memory>
#include <vector>
#include <chrono>

#include "image_proc/rectify_resize_fpga_streamlined.hpp"
#include "tracetools_image_pipeline/tracetools.h"

namespace image_geometry
{

  // From pinhole_camera_model.cpp
  enum DistortionState
  {
    NONE,
    CALIBRATED,
    UNKNOWN
  };

  struct PinholeCameraModelFPGAStreamlined::Cache
  {
    DistortionState distortion_state;

    cv::Mat_<double> K_binned, P_binned; // NOLINT, Binning applied, but not cropping

    mutable bool full_maps_dirty;
    mutable cv::Mat full_map1, full_map2;

    mutable bool reduced_maps_dirty;
    mutable cv::Mat reduced_map1, reduced_map2;

    mutable bool rectified_roi_dirty;
    mutable cv::Rect rectified_roi;

    Cache()
        : full_maps_dirty(true),
          reduced_maps_dirty(true),
          rectified_roi_dirty(true)
    {
    }
  };

  // From pinhole_camera_model.cpp
  PinholeCameraModelFPGAStreamlined::PinholeCameraModelFPGAStreamlined()
  {
  }

  void PinholeCameraModelFPGAStreamlined::rectifyresizeImageFPGA() const
  {
  
  }
} // namespace image_geometry

namespace image_proc
{

  RectifyResizeNodeFPGAStreamlined::RectifyResizeNodeFPGAStreamlined(
      const rclcpp::NodeOptions &options)
      : Node("RectifyResizeNodeFPGAStreamlined", options)
  {
    // Rectify params
    queue_size_ = this->declare_parameter("queue_size", 5);
    interpolation = this->declare_parameter("interpolation", 1);

    // Resize params
    use_scale_ = this->declare_parameter("use_scale", true);
    scale_height_ = this->declare_parameter("scale_height", 1.0);
    scale_width_ = this->declare_parameter("scale_width", 1.0);
    height_ = this->declare_parameter("height", -1);
    width_ = this->declare_parameter("width", -1);
    profile_ = this->declare_parameter("profile", true);
    custom_maps = this->declare_parameter("custom_maps", true);

    pub_rect_ = image_transport::create_publisher(this, "image_rect");
    pub_image_ = image_transport::create_camera_publisher(this, "resize");
    init();
    subscribeToCamera();
  }

  void RectifyResizeNodeFPGAStreamlined::init()
  {
    // Load the acceleration kernel
    // NOTE: hardcoded path according to dfx-mgrd conventions
    cl_int err;
    unsigned fileBufSize;

    std::vector<cl::Device> devices = get_xilinx_devices(); // Get the device:
    cl::Device device = devices[0];
    OCL_CHECK(err, context_ = new cl::Context(device, NULL, NULL, NULL, &err));
    OCL_CHECK(err, queue_ = new cl::CommandQueue(*context_, device, CL_QUEUE_PROFILING_ENABLE | CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE, &err)); // NOLINT
    OCL_CHECK(err, std::string device_name = device.getInfo<CL_DEVICE_NAME>(&err));                                                             // NOLINT
    std::cout << "INFO: Device found - " << device_name << std::endl;

    char *fileBuf = read_binary_file(
        "/lib/firmware/xilinx/kr260-perception/kr260-image-proc-streamlined.xclbin",
        fileBufSize);
    cl::Program::Binaries bins{{fileBuf, fileBufSize}};
    devices.resize(1);
    OCL_CHECK(err, cl::Program program(*context_, devices, bins, NULL, &err));
    OCL_CHECK(err, krnl_rectify = new cl::Kernel(program, "rectify_accel_streamlined", &err));
    OCL_CHECK(err, krnl_resize = new cl::Kernel(program, "resize_accel_streamlined", &err));
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("image_proc");
    std::string tmpx = package_share_directory + "/../mapx.txt";
    std::string tmpy = package_share_directory + "/../mapy.txt";
    std::cout << "Path of image proc package is: " << tmpx.c_str() << "\n";
    custom_map_x.create(480, 640, CV_32FC1);
    custom_map_y.create(480, 640, CV_32FC1);
    FILE *fp_mx, *fp_my;
    fp_mx = fopen(tmpx.c_str(), "r");
    fp_my = fopen(tmpy.c_str(), "r");
    for (int i = 0; i < 480; i++)
    {
      for (int j = 0; j < 640; j++)
      {
        float valx, valy;
        if (fscanf(fp_mx, "%f", &valx) != 1)
        {
          fprintf(stderr, "Not enough data in the provided map_x file ... !!!\n ");
        }
        if (fscanf(fp_my, "%f", &valy) != 1)
        {
          fprintf(stderr, "Not enough data in the provided map_y file ... !!!\n ");
        }
        custom_map_x.at<float>(i, j) = valx;
        custom_map_y.at<float>(i, j) = valy;
      }
    }

    bool gray = 0;
    int rows = 480;
    int cols = 640;
    int channels = gray ? 1 : 3;
     std::cout << "channel is gray? " << channels << "\n";
    image_in_size_bytes = rows * cols * sizeof(unsigned char) * channels;
    map_in_size_bytes = rows * cols * sizeof(float);
    image_out_size_bytes = rows * scale_height_ * cols * scale_width_ * channels * sizeof(unsigned char);
    if (gray)
      hls_remapped.create(cv::Size(cols * scale_width_, rows * scale_height_), CV_8UC1);
    else
      hls_remapped.create(cv::Size(cols * scale_width_, rows * scale_height_), CV_8UC3);

    // cv::Mat tmp;
    // cv::remap( raw, tmp, custom_map_x, custom_map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0) );
    // cv::resize(tmp, hls_remapped, cv::Size(), 1, 1);
    // Allocate the buffers:
    OCL_CHECK(err, cl::Buffer _buffer_inImage(*context_, CL_MEM_READ_ONLY, image_in_size_bytes, NULL, &err));
    OCL_CHECK(err, cl::Buffer _buffer_inMapX(*context_, CL_MEM_READ_ONLY, map_in_size_bytes, NULL, &err));
    OCL_CHECK(err, cl::Buffer _buffer_inMapY(*context_, CL_MEM_READ_ONLY, map_in_size_bytes, NULL, &err));
    OCL_CHECK(err, cl::Buffer _buffer_outImage(*context_, CL_MEM_WRITE_ONLY, image_out_size_bytes, NULL, &err));
    buffer_inImage = _buffer_inImage;
    buffer_inMapX = _buffer_inMapX;
    buffer_inMapY = _buffer_inMapY;
    buffer_outImage = _buffer_outImage;

    // Set kernel arguments:
    std::cout << rows << " x " << cols << "\n";
    std::cout << rows * scale_height_ << " x " << cols * scale_width_ << "\n";
    int output_rows = rows * scale_height_;
    int output_cols = cols * scale_width_;
    OCL_CHECK(err, err = krnl_rectify->setArg(0, buffer_inImage));
    OCL_CHECK(err, err = krnl_rectify->setArg(1, buffer_inMapX));
    OCL_CHECK(err, err = krnl_rectify->setArg(2, buffer_inMapY));
    OCL_CHECK(err, err = krnl_rectify->setArg(4, rows));
    OCL_CHECK(err, err = krnl_rectify->setArg(5, cols));

    OCL_CHECK(err, err = krnl_resize->setArg(1, buffer_outImage));
    OCL_CHECK(err, err = krnl_resize->setArg(2, rows));
    OCL_CHECK(err, err = krnl_resize->setArg(3, cols));
    OCL_CHECK(err, err = krnl_resize->setArg(4, output_rows));
    OCL_CHECK(err, err = krnl_resize->setArg(5, output_cols));

    // We then need to map our OpenCL buffers to get the pointers

    OCL_CHECK(err, ptr_a = (uint8_t *)queue_->enqueueMapBuffer(buffer_inImage, CL_TRUE, CL_MAP_WRITE, 0, image_in_size_bytes, NULL, NULL, &err));
    OCL_CHECK(err, ptr_x = (uint32_t *)queue_->enqueueMapBuffer(buffer_inMapX, CL_TRUE, CL_MAP_WRITE, 0, map_in_size_bytes, NULL, NULL, &err));
    OCL_CHECK(err, ptr_y = (uint32_t *)queue_->enqueueMapBuffer(buffer_inMapY, CL_TRUE, CL_MAP_WRITE, 0, map_in_size_bytes, NULL, NULL, &err));

    OCL_CHECK(err, ptr_result = (uint8_t *)queue_->enqueueMapBuffer(buffer_outImage, CL_TRUE, CL_MAP_READ, 0, image_out_size_bytes, NULL, NULL, &err));
    //cv::Mat raw = cv::imread("/group/xhdwts/mshaik/perception/image_raw.jpg", 1);
    cv::Mat mapx(custom_map_x.size(), custom_map_x.type(), ptr_x);
    custom_map_x.copyTo(mapx);
    cv::Mat mapy(custom_map_y.size(), custom_map_y.type(), ptr_y);
    custom_map_y.copyTo(mapy);

    queue_->finish();
  }

  // Handles (un)subscribing when clients (un)subscribe
  void RectifyResizeNodeFPGAStreamlined::subscribeToCamera()
  {
    std::lock_guard<std::mutex> lock(connect_mutex_);

    sub_camera_ = image_transport::create_camera_subscription(
        this, "image", std::bind(&RectifyResizeNodeFPGAStreamlined::imageCb, this, std::placeholders::_1, std::placeholders::_2), "raw");
  }

  void RectifyResizeNodeFPGAStreamlined::imageCb(
      const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg)
  {

    if (pub_image_.getNumSubscribers() < 1)
    {
      return;
    }

    // Verify camera is actually calibrated
    if (info_msg->k[0] == 0.0)
    {
      RCLCPP_ERROR(
          this->get_logger(), "Rectified topic '%s' requested  "
                              "is uncalibrated",
          pub_image_.getTopic().c_str());
      return;
    }
    // If zero distortion, just pass the message along
    for (size_t i = 0; i < info_msg->d.size(); ++i)
    {
      if (info_msg->d[i] != 0.0)
      {
        zero_distortion = false;
        break;
      }
    }

    // This will be true if D is empty/zero sized
    if (zero_distortion)
    {
      pub_image_.publish(*image_msg, *info_msg);
      return;
    }
   gray = (sensor_msgs::image_encodings::numChannels(image_msg->encoding) == 1);

    /*    if (gray)
          std::cout << "grey image is published from Gazebo \n";
        else
          std::cout << "RGB image is published from Gazebo \n";
    */

    //    std::string package_share_directory = ament_index_cpp::get_package_share_directory("image_proc");
    //    std::cout << "Path of image proc package is: " << package_share_directory << "\n";

    try
    {
      cv_ptr = cv_bridge::toCvCopy(image_msg);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Create cv::Mat views onto both buffers
    const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
    const cv::Mat Image_krnl(image.size(), image.type(), ptr_a);
    image.copyTo(Image_krnl);

    // Prep<are output CameraInfo with desired dimensions
    dst_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(*info_msg);

    double scale_y;
    double scale_x;

    if (use_scale_)
    {
      scale_y = scale_height_;
      scale_x = scale_width_;
      dst_info_msg->height = static_cast<int>(info_msg->height * scale_height_);
      dst_info_msg->width = static_cast<int>(info_msg->width * scale_width_);
    }
    else
    {
      scale_y = static_cast<double>(height_) / info_msg->height;
      scale_x = static_cast<double>(width_) / info_msg->width;
      dst_info_msg->height = height_;
      dst_info_msg->width = width_;
    }

    // Rescale the relevant entries of the intrinsic and extrinsic matrices
    dst_info_msg->k[0] = dst_info_msg->k[0] * scale_x; // fx
    dst_info_msg->k[2] = dst_info_msg->k[2] * scale_x; // cx
    dst_info_msg->k[4] = dst_info_msg->k[4] * scale_y; // fy
    dst_info_msg->k[5] = dst_info_msg->k[5] * scale_y; // cy

    dst_info_msg->p[0] = dst_info_msg->p[0] * scale_x; // fx
    dst_info_msg->p[2] = dst_info_msg->p[2] * scale_x; // cx
    dst_info_msg->p[3] = dst_info_msg->p[3] * scale_x; // T
    dst_info_msg->p[5] = dst_info_msg->p[5] * scale_y; // fy
    dst_info_msg->p[6] = dst_info_msg->p[6] * scale_y; // cy

    // Migrate input data to kernel space
    OCL_CHECK(err, err = queue_->enqueueMigrateMemObjects({buffer_inImage, buffer_inMapX, buffer_inMapY}, 0));

    // Execute the kernel:
    OCL_CHECK(err, err = queue_->enqueueTask(*krnl_rectify));
    OCL_CHECK(err, err = queue_->enqueueTask(*krnl_resize));

    queue_->finish();
    // std::cout << "Copying result buffer data \n";
    OCL_CHECK(err, queue_->enqueueMigrateMemObjects({buffer_outImage}, CL_MIGRATE_MEM_OBJECT_HOST));

    queue_->finish();

    // Set the output image
    output_image.header = cv_ptr->header;
    output_image.encoding = cv_ptr->encoding;

    std::cout << dst_info_msg->height << " x " << dst_info_msg->width << "\n";
    if (gray)
    {
      output_image.image =
          cv::Mat{
              static_cast<int>(dst_info_msg->height),
              static_cast<int>(dst_info_msg->width),
              CV_8UC1,
              ptr_result};
    }
    else
    {
      output_image.image =
          cv::Mat{
              static_cast<int>(dst_info_msg->height),
              static_cast<int>(dst_info_msg->width),
              CV_8UC3,
              ptr_result};
    }

    pub_image_.publish(*output_image.toImageMsg(), *dst_info_msg);
  }

} // namespace image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the
// component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(image_proc::RectifyResizeNodeFPGAStreamlined)
