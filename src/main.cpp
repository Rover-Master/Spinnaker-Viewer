#include "Spinnaker.h" // IWYU pragma: export
#include "spinnaker/wrapper.h"

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
    Spinnaker::ConfigurableMap cameraConfig(camera->GetNodeMap());
    cameraConfig.set<std::string>("AcquisitionMode", "Continuous");
    cameraConfig.set<std::string>("ExposureAuto", "Continuous");
    // cameraConfig.set<double>("ExposureTime", 10.0 * 1000.0);
    cameraConfig.set<bool>("AcquisitionFrameRateEnable", true);
    cameraConfig.set<double>("AcquisitionFrameRate", 24.0);
    cameraConfig.set<std::string>("GainAuto", "Continuous");
  } catch (Spinnaker::Exception &e) {
    cout << "Error Configuring Camera Acquisition: " << e.what() << endl;
  }

  try {
    Spinnaker::ConfigurableMap streamConfig(camera->GetTLStreamNodeMap());
    streamConfig.set<std::string>("StreamMode", "Socket");
    streamConfig.set<std::string>("StreamBufferHandlingMode", "NewestOnly");
  } catch (Spinnaker::Exception &e) {
    cout << "Error Configuring Camera Stream: " << e.what() << endl;
  }

}

// This function acquires and saves 10 images from a device.
void AcquireImages(CameraPtr camera) {
  int result = 0;

  cout << endl << endl << "*** IMAGE ACQUISITION ***" << endl << endl;

  try {
    cout << "Acquiring images..." << endl;
    camera->BeginAcquisition();
    ImageProcessor processor;
    processor.SetColorProcessing(
        SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);
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
          cv::imshow("Camera Feed", img);
        }
        pResultImage->Release();
      } catch (Spinnaker::Exception &e) {
        cout << "Error: " << e.what() << endl;
        result = -1;
      }
    } while (cv::waitKey(1) != 27);
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
    cout << "No Spinnaker Camera found in this system...." << endl;
  else
    cout << "Number of cameras detected: " << numCameras << endl << endl;
  // Run example on each camera
  for (unsigned int i = 0; i < numCameras; i++) {
    try {
      auto camera = camList.GetByIndex(i);
      cout << endl << "Running example for camera " << i << "..." << endl;

      cout << " ========== Device Info ==========" << endl;
      for (auto info : Spinnaker::device_info(camera))
        cout << info.name << ": " << info.value << endl;

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
