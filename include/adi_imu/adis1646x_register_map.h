#ifndef ADIS1646X_REGISTER_MAP_H
#define ADIS1646X_REGISTER_MAP_H

#include "adis_register_map.h"

namespace adi_imu
{

class Adis1646xRegisterMap : public ADISRegisterMap
{
public:
  explicit Adis1646xRegisterMap(adis_device_id device_id);

private:
  void initializeConstants() override;
};

}  // namespace adi_imu

#endif  // ADIS1646X_REGISTER_MAP_H