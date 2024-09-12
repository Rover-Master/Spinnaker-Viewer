#include "common.h"

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/executors.hpp>

int main(int argc, char *const argv[]) {
  try {
    rclcpp::init(argc, argv);
    auto spinnaker = Spinnaker::System::GetInstance();
    auto camList = spinnaker->GetCameras();
    auto node = rclcpp::Node::make_shared("spinnaker_camera");
    CapList caps;
    const unsigned int numCameras = camList.GetSize();
    // Finish if there are no cameras
    if (numCameras == 0) {
      std::cerr << "No Spinnaker Camera found on this system...." << std::endl;
      return 0;
    }
    std::cout << "Number of cameras detected: " << numCameras << std::endl;
    // Acquire image from each camera
    for (unsigned int i = 0; i < numCameras; i++) {
      try {
        auto camera = camList.GetByIndex(i);
        camera->Init();
        auto t = std::make_unique<Capture>(i, camera, node);
        caps.push_back(std::move(t));
      } catch (Spinnaker::Exception &e) {
        std::cerr << "Spinnaker Error: " << e.what() << std::endl;
      } catch (std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
      } catch (...) {
        std::cerr << "Unknown Error" << std::endl;
      }
    }
    std::cout << "All capture caps initialized" << std::endl;
    while (rclcpp::ok())
      for (auto &cap : caps) {
        rclcpp::spin_some(node);
        if (!cap->pipe.next(cap->frame))
          continue;
        auto header = std_msgs::msg::Header();
        header.stamp = cap->frame->stamp;
        auto img =
            cv_bridge::CvImage(header, "bgr8", cap->frame->frame).toImageMsg();
        cap->msg.img->publish(*img);
        F64Msg msg;
        msg.data = cap->fps;
        cap->msg.fps->publish(msg);
        msg.data = cap->gain;
        cap->msg.gain->publish(msg);
        msg.data = cap->exposure;
        cap->msg.exposure->publish(msg);
      }
    std::cout << "Shutting down..." << std::endl;
    rclcpp::shutdown();
    caps.clear();
    camList.Clear();
    spinnaker->ReleaseInstance();
    return 0;
  } catch (std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  } catch (...) {
    std::cerr << "Error: Unknown" << std::endl;
    return 1;
  }
}
