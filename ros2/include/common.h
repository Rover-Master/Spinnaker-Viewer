// =============================================================================
// TODO: Add file description
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================

#pragma once

#include <memory>
#include <thread>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>

#include <spinnaker/spinnaker.h>
#include <threading/leaky.h>

using Node = rclcpp::Node;
using ImgMsg = sensor_msgs::msg::Image;
using F64Msg = std_msgs::msg::Float64;

typedef struct {
  cv::Mat frame;
  rclcpp::Time stamp;
} MatStamped;

class Capture {
private:
  void entry();
  void loop();
  Spinnaker::CameraPtr camera;
  Spinnaker::ImageProcessor processor;

  bool flag_term = false;
  std::unique_ptr<std::thread> thread;

public:
  struct {
    rclcpp::Publisher<ImgMsg>::SharedPtr img;
    rclcpp::Publisher<F64Msg>::SharedPtr fps, gain, exposure, delay;
  } msg;

public:
  threading::LeakyIO<MatStamped> pipe;
  double fps = 0.0;
  double gain = 0.0;
  double exposure = 0.0;
  // For main thread use only
  std::shared_ptr<const MatStamped> frame;
  // Constructor and destructor
  Capture(unsigned int id, Spinnaker::CameraPtr camera, Node::SharedPtr node);
  ~Capture();
};

using CapList = std::vector<std::unique_ptr<Capture>>;
