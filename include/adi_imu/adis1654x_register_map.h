#ifndef ADIS1654X_REGISTER_MAP_H
#define ADIS1654X_REGISTER_MAP_H

#include "adis_register_map.h"

namespace adi_imu
{

class Adis1654xRegisterMap : public ADISRegisterMap
{
public:
  explicit Adis1654xRegisterMap(adis_device_id device_id);

private:
  void initializeConstants() override;
};

}  // namespace adi_imu

#endif  // ADIS1654X_REGISTER_MAP_H
