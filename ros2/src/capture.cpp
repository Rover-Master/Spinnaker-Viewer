#include "threading/exception.h"
#include <common.h>
#include <spinnaker/spinnaker.h>
#include <vector>

using namespace Spinnaker;

CaptureThread::CaptureThread(unsigned int id, Spinnaker::CameraPtr &camera)
    : std::thread(&CaptureThread::entry, this), camera(camera), id(id) {}

CaptureThread::~CaptureThread() {
  flag_term = true;
  join();
}

CaptureThread::config() {
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
    cout << "Error Configuring Camera: " << e.what() << endl;
  }
}

void CaptureThread::entry() {
  camera->init();
  config();
  camera->StartAcquisition();
  processor.SetColorProcessing(SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);
  try {
    while (!flag_term)
      loop();
  }
  EXPECT_END_OF_STREAM
  catch (Spinnaker::Exception &e) {
    cerr << "Spinnaker Error: " << e.what() << endl;
  }
  catch (std::exception &e) {
    cerr << "Error: " << e.what() << endl;
  }
  catch (...) {
    cerr << "Error: Unknown" << endl;
  }
  camera->EndAcquisition();
  camera->DeInit();
}

CaptureThread::loop() {
  try {
    ImagePtr pResultImage = camera->GetNextImage(100);
    if (pResultImage->IsIncomplete()) {
      cerr << "Image incomplete: "
           << Image::GetImageStatusDescription(pResultImage->GetImageStatus())
           << endl;
    } else {
      const size_t width = pResultImage->GetWidth(),
                   height = pResultImage->GetHeight();
      ImagePtr convertedImage =
          processor.Convert(pResultImage, PixelFormat_BGR8);
      // Display image using OpenCV
      cv::Mat img = cv::Mat(height, width, CV_8UC3, convertedImage->GetData(),
                            cv::Mat::AUTO_STEP);
      pipe.write(std::move(img.clone()))
    }
    pResultImage->Release();
  } catch (Spinnaker::Exception &e) {
    cout << "Error: " << e.what() << endl;
  }
}

SystemPtr system = System::GetInstance();
void sys_release() { system->ReleaseInstance(); }

std::vector<CaptureThread> &capture_all() {
  atexit(sys_release);
  auto ret = new std::vector<CaptureThread>();
  CameraList camList = system->GetCameras();
  const unsigned int numCameras = camList.GetSize();
  // Finish if there are no cameras
  if (numCameras == 0)
    cerr << "No Spinnaker Camera found on this system...." << endl;
  // Acquire image from each camera
  for (unsigned int i = 0; i < numCameras; i++) {
    try {
      ret.push_back(CaptureThread(i, std::move(camList.GetByIndex(i))));
    } catch (Spinnaker::Exception &e) {
      cerr << "Error: " << e.what() << endl;
    } catch (std::exception &e) {
      cerr << "Error: " << e.what() << endl;
    } catch (...) {
      cerr << "Error: Unknown" << endl;
    }
  }
  camList.Clear();
  return *ret;
};
