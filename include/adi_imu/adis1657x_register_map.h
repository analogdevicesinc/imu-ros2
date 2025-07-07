#ifndef ADIS1657X_REGISTER_MAP_H
#define ADIS1657X_REGISTER_MAP_H

#include "adis_register_map.h"

namespace adi_imu
{

class Adis1657xRegisterMap : public ADISRegisterMap
{
public:
  explicit Adis1657xRegisterMap(adis_device_id device_id);

private:
  void initializeConstants() override;
  void overwriteRegisters() override;

};

}  // namespace adi_imu

#endif  // ADIS1657X_REGISTER_MAP_H
