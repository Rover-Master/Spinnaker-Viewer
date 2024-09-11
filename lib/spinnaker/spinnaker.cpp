// =============================================================================
// TODO: Add file description
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================

#include <iostream>
#include <limits>
#include <string>

#include "assert.h"
#include "spinnaker/spinnaker.h"

#define NO_NULL(NODE) ASSERT(NODE != nullptr, "Node does not exist");

#define CHECK(VIEW)                                                            \
  ASSERT(VIEW != nullptr, "Node does not exist");                              \
  ASSERT(VIEW.IsValid(), "invalid node name");                                 \
  ASSERT(GenApi::IsAvailable(VIEW), "node not available");                     \
  ASSERT(GenApi::IsWritable(VIEW), "node not writable");

#define CATCH_SET(TYPE, KEY, VALUE)                                            \
  catch (std::exception & e) {                                                 \
    std::cerr << "[Spinnaker::config] Unable to set <" TYPE "> " << KEY        \
              << " = " << VALUE << " (" << e.what() << ")" << std::endl;       \
    return -1;                                                                 \
  }                                                                            \
  return 0;

#define CATCH_GET(TYPE, KEY, RET)                                              \
  catch (std::exception & e) {                                                 \
    std::cerr << "[Spinnaker::config] Unable to get <" TYPE "> " << KEY        \
              << " (" << e.what() << ")" << std::endl;                         \
    return RET;                                                                \
  }

namespace Spinnaker {

std::string Lines::join(const std::string &delim) {
  std::stringstream out;
  bool first = true;
  for (const auto &line : *this) {
    if (!first)
      out << delim;
    else
      first = false;
    out << line;
  }
  return out.str();
}

ConfigurableMap::ConfigurableMap(GenApi::INodeMap &map) : map(map){};

DeviceInfo::DeviceInfo(Spinnaker::CameraPtr &camera)
    : vector<DeviceInfoEntry>() {
  auto &map = camera->GetTLDeviceNodeMap();
  try {
    Spinnaker::GenApi::CStringPtr deviceSerial = map.GetNode("DeviceModelName");
    if (IsReadable(deviceSerial))
      push_back({"Model", deviceSerial->GetValue().c_str()});
    else
      push_back({"Model", "N/A"});
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
        const auto readable = IsReadable(pValue);
        auto value = (readable ? pValue->ToString() : "N/A").c_str();
        if ((*name) || readable)
          push_back({name, value});
      }
    } else {
      push_back({"ERROR", "DeviceInformation node not readable"});
    }
  } catch (Spinnaker::Exception &e) {
    push_back({"ERROR", e.what()});
  }
}

Lines DeviceInfo::format(std::string sep) {
  Lines lines;
  size_t max_len = 0;
  for (const auto &entry : *this)
    max_len = std::max(max_len, entry.name.size());
  for (const auto &entry : *this) {
    auto space = std::string(max_len - entry.name.size(), ' ');
    lines << entry.name + space + sep + entry.value;
  }
  return lines;
}

// ===========================        enum        =============================

/**
 * @brief Get enum entry by given entry name
 * @returns (string) value on success, empty string on failure
 */
template <> std::string ConfigurableMap::get<std::string>(const char *key) {
  try {
    const auto node = this->map.GetNode(key);
    NO_NULL(node);
    const auto view = GenApi::CEnumerationPtr(node);
    CHECK(view);
    const auto entry = view->GetCurrentEntry();
    return std::string(entry->GetSymbolic().c_str());
  }
  CATCH_GET("enum", key, "");
}

/**
 * @brief Set enum entry by given entry name
 * @returns 0 on success, -1 on failure
 */
template <>
int ConfigurableMap::set<std::string>(const char *key,
                                      const std::string value) {
  try {
    const auto node = this->map.GetNode(key);
    NO_NULL(node);
    const auto view = GenApi::CEnumerationPtr(node);
    CHECK(view);
    const auto entry = view->GetEntryByName(value.c_str());
    ASSERT(GenApi::IsAvailable(entry), "enum entry not available");
    ASSERT(GenApi::IsReadable(entry), "enum entry not readable");
    const auto enum_value = entry->GetValue();
    view->SetIntValue(enum_value);
  }
  CATCH_SET("enum", key, value);
}

// ==========================        bool        ==============================

/**
 * @brief Get boolean entry by given value
 * @returns (bool) value on success, 0 (false) on failure
 */
template <> bool ConfigurableMap::get(const char *key) {
  try {
    const auto node = this->map.GetNode(key);
    NO_NULL(node);
    const auto view = GenApi::CBooleanPtr(node);
    CHECK(view);
    return view->GetValue();
  }
  CATCH_GET("bool", key, 0);
}

/**
 * @brief Set boolean entry by given value
 * @returns 0 on success, -1 on failure
 */
template <> int ConfigurableMap::set<bool>(const char *key, const bool value) {
  try {
    const auto node = this->map.GetNode(key);
    NO_NULL(node);
    const auto view = GenApi::CBooleanPtr(node);
    CHECK(view);
    view->SetValue(value);
  }
  CATCH_SET("bool", key, value);
}

// ==========================        int        ===============================

/**
 * @brief Get int entry by given value
 * @returns (int) value on success, -1 on failure
 */
template <> int ConfigurableMap::get<int>(const char *key) {
  try {
    const auto node = this->map.GetNode(key);
    NO_NULL(node);
    const auto view = GenApi::CIntegerPtr(node);
    CHECK(view);
    return view->GetValue();
  }
  CATCH_SET("int", key, -1);
}

/**
 * @brief Set int entry by given value
 * @returns 0 on success, -1 on failure
 */
template <> int ConfigurableMap::set<int>(const char *key, const int value) {
  try {
    const auto node = this->map.GetNode(key);
    NO_NULL(node);
    const auto view = GenApi::CIntegerPtr(node);
    CHECK(view);
    view->SetValue(value);
  }
  CATCH_SET("int", key, value);
}

// ==========================        double        ============================

/**
 * @brief Get double entry by given value
 * @returns (double) value on success, signaling NaN on failure
 */
template <> double ConfigurableMap::get<double>(const char *key) {
  try {
    const auto node = this->map.GetNode(key);
    NO_NULL(node);
    const auto view = GenApi::CFloatPtr(node);
    CHECK(view);
    return view->GetValue();
  }
  CATCH_SET("double", key, std::numeric_limits<double>::signaling_NaN());
}

/**
 * @brief Set double entry by given value
 * @returns 0 on success, -1 on failure
 */
template <>
int ConfigurableMap::set<double>(const char *key, const double value) {
  try {
    const auto node = this->map.GetNode(key);
    NO_NULL(node);
    const auto view = GenApi::CFloatPtr(node);
    CHECK(view);
    view->SetValue(value);
  }
  CATCH_SET("double", key, value);
}

// ===========================    enum options    =============================

/**
 * @brief Get enum options by given entry name
 * @returns (vector<string>) options on success, empty vector on failure
 */
template <> std::vector<std::string> ConfigurableMap::options(const char *key) {
  try {
    const auto node = this->map.GetNode(key);
    NO_NULL(node);
    const auto view = GenApi::CEnumerationPtr(node);
    CHECK(view);
    std::vector<std::string> options;
    GenApi::NodeList_t entries;
    view->GetEntries(entries);
    for (const auto &entry : entries) {
      const auto el = GenApi::CEnumEntryPtr(entry);
      options.push_back(el->GetSymbolic().c_str());
    }
    return options;
  }
  CATCH_GET("enum options", key, {});
}

/**
 * @brief List all options for a given key, highlighting the active value
 * @returns Lines lines of outputs
 */
Lines ConfigurableMap::list(const char *key) {
  Lines lines;
  const auto v = get<string>(key);
  lines << string(key) + ": ";
  for (const auto &opt : options<string>(key))
    lines << string(" - [") + string((opt == v) ? "*" : " ") + string("] ") +
                 opt;
  return lines;
};

}; // namespace Spinnaker
