#include "adi_imu/adis_device_factory.h"

#include "adi_imu/adis1646x_register_map.h"
#include "adi_imu/adis1647x_register_map.h"
#include "adi_imu/adis1650x_register_map.h"
#include "adi_imu/adis1654x_register_map.h"
#include "adi_imu/adis1655x_register_map.h"
#include "adi_imu/adis1657x_register_map.h"
#include "adi_imu/utils/adis_device_registry.h"

namespace adi_imu
{

std::shared_ptr<ADISRegisterMap> ADISDeviceFactory::make(const std::string & device_name)
{
  adis_device_id device_id = ADISDeviceRegistry::getDeviceIdFromString(device_name);
  if (device_id != static_cast<adis_device_id>(-1)) {
    return make(device_id);
  } else {
    throw std::invalid_argument("Unsupported device ID");
  }
}

std::shared_ptr<ADISRegisterMap> ADISDeviceFactory::make(adis_device_id device_id)
{
  auto family = ADISDeviceRegistry::getDeviceFamily(device_id);
  if (family == "adis1646x") {
    return std::make_shared<Adis1646xRegisterMap>(device_id);
  } else if (family == "adis1647x") {
    return std::make_shared<Adis1647xRegisterMap>(device_id);
  } else if (family == "adis1650x") {
    return std::make_shared<Adis1650xRegisterMap>(device_id);
  } else if (family == "adis1654x") {
    return std::make_shared<Adis1654xRegisterMap>(device_id);
  } else if (family == "adis1655x") {
    return std::make_shared<Adis1655xRegisterMap>(device_id);
  } else if (family == "adis1657x") {
    return std::make_shared<Adis1657xRegisterMap>(device_id);
  } else {
    throw std::invalid_argument("Unsupported device family: " + family);
  }
}

}  // namespace adi_imu