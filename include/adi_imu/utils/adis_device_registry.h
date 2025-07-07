#ifndef ADIS_DEVICE_REGISTRY_H
#define ADIS_DEVICE_REGISTRY_H

#include "adi_imu/utils/adis_register_definitions.h"
#include <unordered_map>
#include <string>
#include <vector>
#include <stdexcept>

namespace adi_imu
{

class ADISDeviceRegistry
{
public:
  // Device name <-> ID conversion
  static adis_device_id getDeviceIdFromString(const std::string& device_name);
  static std::string getDeviceNameFromId(adis_device_id device_id);

  // Device queries
  static std::vector<std::string> getSupportedDeviceNames();
  static std::vector<adis_device_id> getSupportedDeviceIds();
  static bool isDeviceSupported(const std::string& device_name);
  static bool isDeviceSupported(adis_device_id device_id);

  // Device family classification
  static std::string getDeviceFamily(adis_device_id device_id);
  static std::string getDeviceFamily(const std::string& device_name);

private:
  static const std::unordered_map<std::string, adis_device_id> m_supported_devices;
  // static const std::unordered_map<adis_device_id, std::string> m_device_families;
};

}  // namespace adi_imu

#endif  // ADIS_DEVICE_REGISTRY_H