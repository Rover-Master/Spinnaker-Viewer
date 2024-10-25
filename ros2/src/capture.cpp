#include "spinnaker/spinnaker.h"
#include <common.h>

using namespace std;
using namespace Spinnaker;

Capture::Capture(unsigned int id, CameraPtr camera, Node::SharedPtr node)
    : camera(camera) {
  cout << "Camera constructing" << endl;
  string topic = "camera_" + to_string(id);
  msg.img = node->create_publisher<ImgMsg>(topic + "/img", 10);
  msg.fps = node->create_publisher<F64Msg>(topic + "/fps", 10);
  msg.gain = node->create_publisher<F64Msg>(topic + "/gain", 10);
  msg.exposure = node->create_publisher<F64Msg>(topic + "/exposure", 10);
  msg.delay = node->create_publisher<F64Msg>(topic + "/delay", 10);
  cout << "Starting capture thread" << endl;
  thread = make_unique<std::thread>(&Capture::entry, this);
  RCLCPP_INFO(node->get_logger(), "Camera %u started", id);
}

Capture::~Capture() {
  flag_term = true;
  if (thread && thread->joinable())
    thread->join();
}

void Capture::entry() {
  cout << "Capture thread started" << endl;
  camera->Init();
  cout << "Camera initialized" << endl;
  try {
    ConfigurableMap map(camera->GetNodeMap());
    map.set<string>("AcquisitionMode", "Continuous");
    map.set<string>("ExposureAuto", "Continuous");
    // map.set<double>("ExposureTime", 10.0 * 1000.0);
    map.set<bool>("AcquisitionFrameRateEnable", true);
    map.set<double>("AcquisitionFrameRate", 20.0);
    map.set<string>("GainAuto", "Off");
    map.set<double>("Gain", 20.0);
  } catch (Exception &e) {
    cout << "Error Configuring Camera: " << e.what() << endl;
  } catch (...) {
    cout << "Error Configuring Camera: [Unknown]" << endl;
  }
  cout << "Camera config complete" << endl;
  camera->BeginAcquisition();
  processor.SetColorProcessing(SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);
  cout << "Entering capture loop" << endl;
  try {
    while (!flag_term)
      loop();
  }
  EXPECT_END_OF_STREAM
  catch (Exception &e) {
    cerr << "Spinnaker Error: " << e.what() << endl;
  }
  catch (exception &e) {
    cerr << "Error: " << e.what() << endl;
  }
  catch (...) {
    cerr << "Error: Unknown" << endl;
  }
  camera->EndAcquisition();
  camera->DeInit();
}

void Capture::loop() {
  try {
    ConfigurableMap map(camera->GetNodeMap());
    fps = map.get<double>("AcquisitionFrameRate");
    gain = map.get<double>("Gain");
    exposure = map.get<double>("ExposureTime");
    ImagePtr pResultImage = camera->GetNextImage(1000);
    auto stamp = rclcpp::Clock().now();
    if (pResultImage->IsIncomplete()) {
      cerr << "Image incomplete: "
           << Image::GetImageStatusDescription(pResultImage->GetImageStatus())
           << endl;
    } else {
      const size_t width = pResultImage->GetWidth(),
                   height = pResultImage->GetHeight();
      ImagePtr convertedImage =
          processor.Convert(pResultImage, PixelFormat_BGR8);
      cv::Mat img = cv::Mat(height, width, CV_8UC3, convertedImage->GetData(),
                            cv::Mat::AUTO_STEP);
      pipe.write(MatStamped{std::move(img.clone()), stamp});
    }
    pResultImage->Release();
  } catch (Exception &e) {
    cout << "Error: " << e.what() << endl;
  }
}
