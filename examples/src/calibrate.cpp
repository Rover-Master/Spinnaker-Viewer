// =============================================================================
// TODO: Add file description
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================

#include "spinnaker/spinnaker.h"
#include "threading/exception.h"

#include <exception>
#include <iostream>
#include <memory>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <sstream>
#include <thread>
#include <threading/leaky.h>

using namespace std;
using namespace chrono_literals;
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

// Thread synchronization
bool flag_term;
threading::LeakyIO<cv::Mat> img_pipe, disp_pipe;

// This function demonstrates how we can change stream modes.
string configure(CameraPtr &camera) {
  try {
    Spinnaker::ConfigurableMap config(camera->GetNodeMap());
    config.set<string>("AcquisitionMode", "Continuous");
    config.set<string>("ExposureAuto", "Continuous");
    // config.set<double>("ExposureTime", 10.0 * 1000.0);
    config.set<bool>("AcquisitionFrameRateEnable", true);
    config.set<double>("AcquisitionFrameRate", 24.0);
    config.set<string>("GainAuto", "Continuous");
  } catch (Spinnaker::Exception &e) {
    cout << "Error Configuring Camera Acquisition: " << e.what() << endl;
  }
  try {
    Spinnaker::ConfigurableMap config(camera->GetTLStreamNodeMap());
    config.set<string>("StreamBufferHandlingMode", "NewestOnly");
  } catch (Spinnaker::Exception &e) {
    cout << "Error Configuring Camera Stream: " << e.what() << endl;
  }
  // Get camera name
  auto &map = camera->GetTLDeviceNodeMap();
  Spinnaker::GenApi::CStringPtr deviceSerial = map.GetNode("DeviceModelName");
  string name = IsReadable(deviceSerial) ? deviceSerial->GetValue().c_str()
                                         : "Unknown Camera Model";
  return name;
}

// This function acquires and saves 10 images from a device.
void acquire(CameraPtr camera, string name) {
  try {
    camera->BeginAcquisition();
    ImageProcessor processor;
    processor.SetColorProcessing(
        SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);
    cout << "Acquiring images from " << name << " ..." << endl;
    while (!flag_term) {
      try {
        ImagePtr pResultImage = camera->GetNextImage(100);
        if (pResultImage->IsIncomplete()) {
          // Retrieve and print the image status description
          cout << "Image incomplete: "
               << Image::GetImageStatusDescription(
                      pResultImage->GetImageStatus())
               << endl;
        } else {
          const size_t width = pResultImage->GetWidth(),
                       height = pResultImage->GetHeight();
          ImagePtr convertedImage =
              processor.Convert(pResultImage, PixelFormat_BGR8);
          cv::Mat mat(height, width, CV_8UC3, convertedImage->GetData(),
                      cv::Mat::AUTO_STEP);
          img_pipe.write(mat.clone());
        }
        pResultImage->Release();
      } catch (Spinnaker::Exception &e) {
        cout << "Error: " << e.what() << endl;
      }
    }
    camera->EndAcquisition();
    cv::destroyAllWindows();
  } catch (Spinnaker::Exception &e) {
    cout << "Error During Acquisition: " << e.what() << endl;
  }
  EXPECT_END_OF_STREAM
}

static int M = 4, N = 4;
// time of last pattern found
chrono::time_point<chrono::system_clock> last_record;
void detect(unsigned int camera_id) {
  try {
    vector<vector<cv::Point2f>> img_points;
    vector<vector<cv::Point3f>> obj_points;
    vector<cv::Point3f> obj;
    vector<cv::Point2f> corners;
    cv::Size img_size;
    obj.reserve(M * N);
    for (int i = 0; i < N; i++)
      for (int j = 0; j < M; j++)
        obj.push_back(cv::Point3f(i, j, 0));
    shared_ptr<const cv::Mat> img;
    while (!flag_term) {
      if (!img_pipe.next(img, true))
        continue;
      // Detect checkerboard
      cv::Mat gray;
      cv::cvtColor(*img, gray, cv::COLOR_BGR2GRAY);
      img_size = gray.size();
      bool found = cv::findChessboardCorners(gray, {M, N}, corners,
                                             cv::CALIB_CB_ADAPTIVE_THRESH +
                                                 cv::CALIB_CB_NORMALIZE_IMAGE +
                                                 cv::CALIB_CB_FAST_CHECK);
      cv::Mat disp(img->clone());
      cv::putText(disp,
                  string("Got " + to_string(img_points.size()) + " samples"),
                  {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);
      if (found) {
        auto criteria = cv::TermCriteria(
            cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1);
        cv::cornerSubPix(gray, corners, {11, 11}, {-1, -1}, criteria);
        cv::drawChessboardCorners(disp, {M, N}, corners, found);
      }
      disp_pipe.write(std::move(disp));
      if (!found)
        continue;
      auto now = chrono::system_clock::now();
      if (now > last_record + 1s) {
        img_points.push_back(corners);
        obj_points.push_back(obj);
        last_record = now;
      }
    }
    // Calibrate camera
    if (img_points.size() > 0) {
      cv::Mat mtx, dist;
      vector<cv::Mat> rv, tv;
      cv::calibrateCamera(obj_points, img_points, img_size, mtx, dist, rv, tv);
      cout << "========== Camera Matrix ==========" << endl << mtx << endl;
      cout << "===== Distortion Coefficients =====" << endl << dist << endl;
      // Save as yaml
      stringstream ss;
      ss << "camera_" << camera_id << ".yaml";
      cv::FileStorage fs(ss.str(), cv::FileStorage::WRITE);
      fs << "mtx" << mtx;
      fs << "dist" << dist;
    }
  }
  EXPECT_END_OF_STREAM
  catch (cv::Exception &e) {
    cout << "OpenCV Error: " << e.what() << endl;
  }
  catch (exception &e) {
    cout << "Error: " << e.what() << endl;
  }
  catch (...) {
    cout << "Unknown Error" << endl;
  }
}

void disp() {
  shared_ptr<const cv::Mat> img;
  while (!flag_term) {
    if (!disp_pipe.next(img, true))
      continue;
    cv::imshow("Image", *img);
    if (cv::waitKey(1) >= 0)
      flag_term = true;
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
      flag_term = false;
      auto camera = camList.GetByIndex(i);
      camera->Init();
      auto name = configure(camera);
      auto acquire_thread = thread(&acquire, camera, name);
      auto detect_thread = thread(&detect, i);
      disp();
      detect_thread.join();
      acquire_thread.join();
      camera->DeInit();
    } catch (Spinnaker::Exception &e) {
      cout << "Error: " << e.what() << endl;
    }
  }
  camList.Clear();
  system->ReleaseInstance();
  return 0;
}
