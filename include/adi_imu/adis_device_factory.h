#ifndef ADIS_DEVICE_FACTORY_H
#define ADIS_DEVICE_FACTORY_H

#include <memory>
#include <string>
#include <vector>

#include "adi_imu/adis_register_map.h"
#include "adi_imu/utils/adis_register_definitions.h"

namespace adi_imu
{

class ADISDeviceFactory
{
public:
  static std::shared_ptr<ADISRegisterMap> make(adis_device_id device_id);
  static std::shared_ptr<ADISRegisterMap> make(const std::string & device_name);
};

}  // namespace adi_imu

#endif  // ADIS_DEVICE_FACTORY_H