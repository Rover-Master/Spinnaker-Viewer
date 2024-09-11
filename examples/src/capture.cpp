// =============================================================================
// TODO: Add file description
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================

#include "spinnaker/spinnaker.h"

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

// This function demonstrates how we can change stream modes.
void configure(CameraPtr camera) {
  try {
    Spinnaker::ConfigurableMap config(camera->GetNodeMap());
    config.set<string>("AcquisitionMode", "Continuous");
    config.set<string>("ExposureAuto", "Continuous");
    // config.set<double>("ExposureTime", 10.0 * 1000.0);
    config.set<bool>("AcquisitionFrameRateEnable", true);
    config.set<double>("AcquisitionFrameRate", 24.0);
    config.set<string>("GainAuto", "Continuous");
    // Print Acquisition Configurations
    cout << config.list("AcquisitionMode").join() << endl
         << config.list("ExposureAuto").join() << endl
         << config.info<double>("ExposureTime") << endl
         << config.info<bool>("AcquisitionFrameRateEnable") << endl
         << config.info<double>("AcquisitionFrameRate") << endl
         << config.list("GainAuto").join() << endl;
  } catch (Spinnaker::Exception &e) {
    cout << "Error Configuring Camera Acquisition: " << e.what() << endl;
  }
  try {
    Spinnaker::ConfigurableMap config(camera->GetTLStreamNodeMap());
    config.set<string>("StreamBufferHandlingMode", "NewestOnly");
    // Print Stream Configurations
    cout << config.list("StreamBufferHandlingMode").join() << endl;
  } catch (Spinnaker::Exception &e) {
    cout << "Error Configuring Camera Stream: " << e.what() << endl;
  }
}

// This function acquires and saves 10 images from a device.
void AcquireImages(CameraPtr camera) {
  try {
    camera->BeginAcquisition();
    ImageProcessor processor;
    processor.SetColorProcessing(
        SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);
    auto &map = camera->GetTLDeviceNodeMap();
    Spinnaker::GenApi::CStringPtr deviceSerial = map.GetNode("DeviceModelName");
    std::string name = IsReadable(deviceSerial)
                           ? deviceSerial->GetValue().c_str()
                           : "Unknown Camera Model";

    cout << "Acquiring images from " << name << " ..." << endl;
    do {
      try {
        ImagePtr pResultImage = camera->GetNextImage(100);
        if (pResultImage->IsIncomplete()) {
          // Retrieve and print the image status description
          cout << "Image incomplete: "
               << Image::GetImageStatusDescription(
                      pResultImage->GetImageStatus())
               << "..." << endl
               << endl;
        } else {
          const size_t width = pResultImage->GetWidth(),
                       height = pResultImage->GetHeight();
          ImagePtr convertedImage =
              processor.Convert(pResultImage, PixelFormat_BGR8);
          // Display image using OpenCV
          cv::Mat img = cv::Mat(height, width, CV_8UC3,
                                convertedImage->GetData(), cv::Mat::AUTO_STEP);
          cv::imshow(name, img);
        }
        pResultImage->Release();
      } catch (Spinnaker::Exception &e) {
        cout << "Error: " << e.what() << endl;
      }
    } while (cv::waitKey(1) < 0);
    camera->EndAcquisition();
    cv::destroyAllWindows();
  } catch (Spinnaker::Exception &e) {
    cout << "Error During Acquisition: " << e.what() << endl;
  }
}

int main(int argc, char *const argv[]) {
  SystemPtr system = System::GetInstance();
  CameraList camList = system->GetCameras();
  const unsigned int numCameras = camList.GetSize();
  // Finish if there are no cameras
  if (numCameras == 0)
    cout << "No Spinnaker Camera found on this system...." << endl;
  // Acquire image from each camera
  for (unsigned int i = 0; i < numCameras; i++) {
    try {
      auto camera = camList.GetByIndex(i);
      cout << endl << "Querying camera <" << i << "> ..." << endl;
      cout << " ========== Device Info ==========" << endl;
      DeviceInfo info(camera);
      cout << info.format(": ").join() << endl;
      cout << " ========== Device Conf ==========" << endl;
      camera->Init();
      configure(camera);

      AcquireImages(camera);
      camera->DeInit();
    } catch (Spinnaker::Exception &e) {
      cout << "Error: " << e.what() << endl;
    }
  }
  camList.Clear();
  system->ReleaseInstance();
  return 0;
}
