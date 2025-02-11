/*******************************************************************************
 *   @file   ros_task.h
 *   @brief  Header for ros imu task.
 *   @author Vasile Holonec (Vasile.Holonec@analog.com)
 *******************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

#ifndef ADIS_DATA_H
#define ADIS_DATA_H

#include <string>
#include <map>

/**
 * @brief Interface for running a task in a thread.
 */
class AdisData
{
public:
  AdisData();
  static AdisData* singleton_;

public:
    AdisData(AdisData &other) = delete;
    void operator=(const AdisData &) = delete;
    static AdisData *GetInstance();
public:
  ~AdisData();

  void setDeviceName(std::string devname);
  int32_t getDeviceValue(std::string key);
  bool checkDeviceName(std::string devname);

private:
  std::map<std::string, int32_t> m_adis1650x_data;
  std::map<std::string, int32_t> m_adis1646x_data;
  std::map<std::string, int32_t> m_adis1647x_data;
  std::map<std::string, int32_t> m_adis1654x_data;
  std::map<std::string, int32_t> m_adis1655x_data;
  std::map<std::string, int32_t> m_adis1657x_data;

  std::map<std::string, std::map<std::string, int32_t> > m_devicesMap;
  std::string m_deviceName;
};


#endif  // ADIS_DATA_H
