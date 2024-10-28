// =============================================================================
// TODO: Add file description
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================

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
#include <sstream>
#include <string>
#include <vector>

namespace Spinnaker {

using namespace std;

class Lines : public vector<string> {
public:
  inline Lines &operator<<(const string str) {
    push_back(str);
    return *this;
  }
  string join(const string &delim = "\n");
};

typedef struct DeviceInfoEntry_s {
  string name, value;
} DeviceInfoEntry;

class DeviceInfo : public vector<DeviceInfoEntry> {
public:
  DeviceInfo() = default;
  DeviceInfo(const DeviceInfo &info) = default;
  DeviceInfo(DeviceInfo &&info) = default;
  DeviceInfo &operator=(const DeviceInfo &info) = default;
  DeviceInfo &operator=(DeviceInfo &&info) = default;
  DeviceInfo(Spinnaker::CameraPtr &camera);
  Lines format(string sep = ": ");
};

class ConfigurableMap {
  GenApi::INodeMap &map;

public:
  ConfigurableMap(GenApi::INodeMap &map);
  template <typename T> int set(const char *key, const T value);
  template <typename T> T get(const char *key);
  template <typename T = string> vector<T> options(const char *key);
  // Pretty print camera config
  Lines list(const char *key);
  template <typename T> string info(const char *key) {
    stringstream ss;
    ss << key << ": " << get<T>(key);
    return string(ss.str());
  };
};

} // namespace Spinnaker
