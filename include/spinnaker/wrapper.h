#pragma once

// Spinnaker defines "interface" as a macro, which conflicts with the
// interface keyword in C++.
// Any user code that uses interface as a variable or class name name will mess
// up.
// ==============================================================================
// located here: spinnaker/include/SpinGenApi/Types.h:24:19
#ifdef interface
#undef interface
#endif
#include <Spinnaker.h> // IWYU pragma: export
#ifdef interface
#undef interface
#endif
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace Spinnaker {

typedef struct DeviceInfoEntry_s {
  std::string name, value;
} DeviceInfoEntry;

typedef std::vector<DeviceInfoEntry> DeviceInfo;

DeviceInfo device_info(Spinnaker::CameraPtr &camera);

class ConfigurableMap {
  GenApi::INodeMap &map;

public:
  ConfigurableMap(GenApi::INodeMap &map);
  template <typename T> int set(const char *key, const T value);
};

} // namespace Spinnaker
