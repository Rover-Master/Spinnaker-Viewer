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
    std::cout << "All capture threads initialized" << std::endl;
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      for (auto &cap : caps) {
        if (!cap->pipe.next(cap->frame))
          continue;
        if (!rclcpp::ok())
          break;
        auto header = std_msgs::msg::Header();
        header.stamp = cap->frame->stamp;
        cv_bridge::CvImage img(header, "bgr8", cap->frame->frame);
        cap->msg.img->publish(*img.toImageMsg());
        // Calculate delay since image grabbed
        static F64Msg msg;
        msg.data = cap->fps;
        cap->msg.fps->publish(msg);
        msg.data = cap->gain;
        cap->msg.gain->publish(msg);
        msg.data = cap->exposure;
        cap->msg.exposure->publish(msg);
        msg.data =
            (rclcpp::Clock().now() - cap->frame->stamp).seconds() * 1000.0;
        cap->msg.delay->publish(msg);
      }
    }
    std::cout << "Terminating ..." << std::endl;
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
