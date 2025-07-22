#ifndef ADIS_REGISTER_MAP_H
#define ADIS_REGISTER_MAP_H

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "adi_imu/utils/adis_register_definitions.h"

namespace adi_imu
{

class ADISRegisterMap
{
protected:
  ADISRegisterMap(adis_device_id device_id);

public:
  void initialize();

  virtual ~ADISRegisterMap() = default;

  bool has(ADISRegister reg) const;
  uint32_t get(ADISRegister reg) const;

  bool hasDeltaBurst() const;
  std::string getDeviceFamily() const;
  std::string getDeviceName() const;
  adis_device_id getDeviceID() const;
  void log() const;

protected:
  void set(ADISRegister reg, uint32_t value);

  virtual void initSharedRegisters() final;
  virtual void computeBitMasks() final;
  virtual void postComputeBitmask() final;
  virtual void initializeConstants() = 0;
  virtual void overwriteRegisters(){};

protected:
  std::unordered_map<ADISRegister, uint32_t> m_register_map;

  adis_device_id m_device_id;
  std::string m_device_name;
  std::string m_device_family;
};

}  // namespace adi_imu

#endif  // ADIS_REGISTER_MAP_H