// Copyright 2025 Analog Devices, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "adi_imu/utils/adis_device_registry.h"

#include <algorithm>
#include <iostream>

namespace adi_imu
{

const std::unordered_map<std::string, adis_device_id> ADISDeviceRegistry::m_supported_devices = {
  {"adis16465-1", ADIS16465_1}, {"adis16465-2", ADIS16465_2}, {"adis16465-3", ADIS16465_3},
  {"adis16467-1", ADIS16467_1}, {"adis16467-2", ADIS16467_2}, {"adis16467-3", ADIS16467_3},
  {"adis16470", ADIS16470},     {"adis16475-1", ADIS16475_1}, {"adis16475-2", ADIS16475_2},
  {"adis16475-3", ADIS16475_3}, {"adis16477-1", ADIS16477_1}, {"adis16477-2", ADIS16477_2},
  {"adis16477-3", ADIS16477_3}, {"adis16500", ADIS16500},     {"adis16501", ADIS16501},
  {"adis16505-1", ADIS16505_1}, {"adis16505-2", ADIS16505_2}, {"adis16505-3", ADIS16505_3},
  {"adis16507-1", ADIS16507_1}, {"adis16507-2", ADIS16507_2}, {"adis16507-3", ADIS16507_3},
  {"adis16545-1", ADIS16545_1}, {"adis16545-2", ADIS16545_2}, {"adis16545-3", ADIS16545_3},
  {"adis16547-1", ADIS16547_1}, {"adis16547-2", ADIS16547_2}, {"adis16547-3", ADIS16547_3},
  {"adis16550", ADIS16550},     {"adis16575-2", ADIS16575_2}, {"adis16575-3", ADIS16575_3},
  {"adis16576-2", ADIS16576_2}, {"adis16576-3", ADIS16576_3}, {"adis16577-2", ADIS16577_2},
  {"adis16577-3", ADIS16577_3}};

adis_device_id ADISDeviceRegistry::getDeviceIdFromString(const std::string & device_name)
{
  std::string lower_name = device_name;
  std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(), ::tolower);

  auto it = m_supported_devices.find(lower_name);
  if (it == m_supported_devices.end()) {
    throw std::invalid_argument("Unsupported device name: " + device_name);
  }
  return it->second;
}

std::string ADISDeviceRegistry::getDeviceNameFromId(adis_device_id device_id)
{
  for (const auto & pair : m_supported_devices) {
    if (pair.second == device_id) {
      return pair.first;
    }
  }
  throw std::invalid_argument(
    "Unsupported device ID: " + std::to_string(static_cast<int>(device_id)));
}

std::vector<std::string> ADISDeviceRegistry::getSupportedDeviceNames()
{
  std::vector<std::string> names;
  names.reserve(m_supported_devices.size());
  for (const auto & pair : m_supported_devices) {
    names.push_back(pair.first);
  }
  std::sort(names.begin(), names.end());
  return names;
}

std::vector<adis_device_id> ADISDeviceRegistry::getSupportedDeviceIds()
{
  std::vector<adis_device_id> ids;
  ids.reserve(m_supported_devices.size());
  for (const auto & pair : m_supported_devices) {
    ids.push_back(pair.second);
  }
  return ids;
}

bool ADISDeviceRegistry::isDeviceSupported(const std::string & device_name)
{
  return m_supported_devices.find(device_name) != m_supported_devices.end();
}

bool ADISDeviceRegistry::isDeviceSupported(adis_device_id device_id)
{
  auto device_name = getDeviceNameFromId(device_id);
  return isDeviceSupported(device_name);
}

std::string ADISDeviceRegistry::getDeviceFamily(adis_device_id device_id)
{
  switch (device_id) {
    // ADIS1646x family
    case ADIS16465_1:
    case ADIS16465_2:
    case ADIS16465_3:
    case ADIS16467_1:
    case ADIS16467_2:
    case ADIS16467_3:
    case ADIS16470:
    case ADIS16475_1:
    case ADIS16475_2:
    case ADIS16475_3:
      return "adis1646x";

    // ADIS1647x family
    case ADIS16477_1:
    case ADIS16477_2:
    case ADIS16477_3:
      return "adis1647x";

    // ADIS1650x family
    case ADIS16500:
    case ADIS16501:
    case ADIS16505_1:
    case ADIS16505_2:
    case ADIS16505_3:
    case ADIS16507_1:
    case ADIS16507_2:
    case ADIS16507_3:
      return "adis1650x";

    // ADIS1654x family
    case ADIS16545_1:
    case ADIS16545_2:
    case ADIS16545_3:
    case ADIS16547_1:
    case ADIS16547_2:
    case ADIS16547_3:
      return "adis1654x";

    // ADIS1655x family
    case ADIS16550:
      return "adis1655x";

    // ADIS1657x family
    case ADIS16575_2:
    case ADIS16575_3:
    case ADIS16576_2:
    case ADIS16576_3:
    case ADIS16577_2:
    case ADIS16577_3:
      return "adis1657x";

    // Unknown device
    default:
      return nullptr;
  }
}

std::string ADISDeviceRegistry::getDeviceFamily(const std::string & device_name)
{
  adis_device_id device_id = getDeviceIdFromString(device_name);
  return getDeviceFamily(device_id);
}

}  // namespace adi_imu
