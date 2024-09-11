// =============================================================================
// TODO: Add file description
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================

#pragma once

#include "CameraPtr.h"
#include <memory>
#include <thread>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>

#include <spinnaker/spinnaker.h>
#include <threading/leaky.h>

using ImageMsg = sensor_msgs::msg::Image;

class CaptureThread : public std::thread {
private:
  bool flag_term = false;
  void config();
  void entry();
  void loop();
  Spinnaker::CameraPtr camera;
  Spinnaker::ImageProcessor processor;

public:
  unsigned int id;
  threading::LeakyIO<cv::Mat> pipe;
  double fps = 0.0;
  double gain = 0.0;
  double exposure = 0.0;
  // For main thread use only
  std::shared_ptr<cv::Mat> frame;
  // Constructor and destructor
  CaptureThread(unsigned int id, Spinnaker::CameraPtr &camera);
  ~CaptureThread();
};

std::vector<CaptureThread> capture_all();
