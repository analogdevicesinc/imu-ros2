#ifndef ADIS1655X_REGISTER_MAP_H
#define ADIS1655X_REGISTER_MAP_H

#include "adis_register_map.h"

namespace adi_imu
{

class Adis1655xRegisterMap : public ADISRegisterMap
{
public:
  explicit Adis1655xRegisterMap(adis_device_id device_id);

private:
  void initializeConstants() override;
  void overwriteRegisters() override;
};

}  // namespace adi_imu

#endif  // ADIS1655X_REGISTER_MAP_H
