#include <iostream>
#include <string>

#include "spinnaker/wrapper.h"
#include "util/assert.h"

#define CHECK(NODE_PTR)                                                        \
  ASSERT(NODE_PTR.IsValid(), "invalid node name");                             \
  ASSERT(GenApi::IsAvailable(NODE_PTR), "node not available");                 \
  ASSERT(GenApi::IsWritable(NODE_PTR), "node not writable");

#define CATCH(NAME, KEY, VALUE)                                                \
  catch (std::exception & e) {                                                 \
    std::cerr << "[Spinnaker::config] Unable to set <" NAME "> " << KEY        \
              << " = " << VALUE << " (" << e.what() << ")" << std::endl;       \
    return 0;                                                                  \
  }                                                                            \
  return 1;

namespace Spinnaker {

ConfigurableMap::ConfigurableMap(GenApi::INodeMap &map) : map(map){};

DeviceInfo device_info(Spinnaker::CameraPtr &camera) {
  auto &map = camera->GetTLDeviceNodeMap();
  DeviceInfo info;
  try {
    Spinnaker::GenApi::FeatureList_t features;
    const Spinnaker::GenApi::CCategoryPtr category =
        map.GetNode("DeviceInformation");
    if (IsReadable(category)) {
      category->GetFeatures(features);
      for (auto feature = features.begin(); feature != features.end();
           ++feature) {
        const Spinnaker::GenApi::CNodePtr pfeatureNode = *feature;
        auto name = pfeatureNode->GetName().c_str();
        Spinnaker::GenApi::CValuePtr pValue =
            static_cast<Spinnaker::GenApi::CValuePtr>(pfeatureNode);
        auto value =
            (IsReadable(pValue) ? pValue->ToString() : "(Not Readable)")
                .c_str();
        info.push_back({name, value});
      }
    } else {
      std::cout << "Device control information not available." << std::endl;
    }
    Spinnaker::GenApi::CStringPtr deviceSerial =
        map.GetNode("DeviceSerialNumber");
    if (IsReadable(deviceSerial))
      info.push_back({"SerialNumber", deviceSerial->GetValue().c_str()});
    else
      info.push_back({"SerialNumber", "(Not Readable)"});
  } catch (Spinnaker::Exception &e) {
    std::cout << "Error Getting Device Info: " << e.what() << std::endl;
  }
  return info;
}

/**
 * @brief Set enum entry by given entry name
 * @returns 1 on success, 0 on failure
 */
template <>
int ConfigurableMap::set<std::string>(const char *key,
                                      const std::string value) {
  try {
    const auto node = this->map.GetNode(key);
    const auto view = GenApi::CEnumerationPtr(node);
    CHECK(view);
    const auto entry = view->GetEntryByName(value.c_str());
    ASSERT(GenApi::IsAvailable(entry), "enum entry not available");
    ASSERT(GenApi::IsReadable(entry), "enum entry not readable");
    const auto enum_value = entry->GetValue();
    view->SetIntValue(enum_value);
  }
  CATCH("enum", key, value);
}

/**
 * @brief Set boolean entry by given value
 * @returns 1 on success, 0 on failure
 */
template <> int ConfigurableMap::set<bool>(const char *key, const bool value) {
  try {
    const auto node = this->map.GetNode(key);
    const auto view = GenApi::CBooleanPtr(node);
    CHECK(view);
    view->SetValue(value);
  }
  CATCH("bool", key, value);
}

/**
 * @brief Set int entry by given value
 * @returns 1 on success, 0 on failure
 */
template <> int ConfigurableMap::set<int>(const char *key, const int value) {
  try {
    const auto node = this->map.GetNode(key);
    const auto view = GenApi::CIntegerPtr(node);
    CHECK(view);
    view->SetValue(value);
  }
  CATCH("int", key, value);
}

/**
 * @brief Set double entry by given value
 * @returns 1 on success, 0 on failure
 */
template <>
int ConfigurableMap::set<double>(const char *key, const double value) {
  try {
    const auto node = this->map.GetNode(key);
    const auto view = GenApi::CFloatPtr(node);
    CHECK(view);
    view->SetValue(value);
  }
  CATCH("double", key, value);
}

}; // namespace Spinnaker
