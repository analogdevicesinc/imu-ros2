/*******************************************************************************
 *   @file   imu_control_parameters.h
 *   @brief  Set ros parameter in libiio
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

#ifndef IMU_CONTROL_PARAMETERS_H
#define IMU_CONTROL_PARAMETERS_H

#include <rclcpp/rclcpp.hpp>

#include "iio_wrapper.h"
#include "imu_ros2/ros_task.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

class ImuControlParameters : public RosTask
{
public:
  ImuControlParameters(std::shared_ptr<rclcpp::Node> & node);
  ~ImuControlParameters();

  void declareFunctions();

  void init(std::shared_ptr<rclcpp::Node> & node);

  void setParametersDouble();
  void setParametersInt32();
  void setParametersUint32();
  void handleCommand();
  void updateRosParams();
  void readDriverParams();

  void run() override;

private:
  std::shared_ptr<rclcpp::Node> m_node;
  IIOWrapper m_iio_wrapper;

  std::string m_command_to_execute;

  typedef bool (IIOWrapper::*UpdateUint32Params)(uint32_t);
  typedef std::map<std::string, UpdateUint32Params> UpdateUint32ParamsMapType;
  UpdateUint32ParamsMapType m_func_map_update_uint32_params;

  typedef bool (IIOWrapper::*GetUint32Params)(uint32_t &);
  typedef std::map<std::string, GetUint32Params> GetUint32ParamsMapType;
  GetUint32ParamsMapType m_func_map_get_uint32_params;

  typedef bool (IIOWrapper::*UpdateInt32Params)(int32_t);
  typedef std::map<std::string, UpdateInt32Params> UpdateInt32ParamsMapType;
  UpdateInt32ParamsMapType m_func_map_update_int32_params;

  typedef bool (IIOWrapper::*GetInt32Params)(int32_t &);
  typedef std::map<std::string, GetInt32Params> GetInt32ParamsMapType;
  GetInt32ParamsMapType m_func_map_get_int32_params;

  typedef bool (IIOWrapper::*UpdateDoubleParams)(double);
  typedef std::map<std::string, UpdateDoubleParams> UpdateDoubleParamsMapType;
  UpdateDoubleParamsMapType m_func_map_update_double_params;

  typedef bool (IIOWrapper::*GetDoubleParams)(double *);
  typedef std::map<std::string, GetDoubleParams> GetDoubleParamsMapType;
  GetDoubleParamsMapType m_func_map_get_double_params;

  typedef bool (IIOWrapper::*ExecuteCommands)();
  typedef std::map<std::string, ExecuteCommands> ExecuteCommandsMapType;
  ExecuteCommandsMapType m_func_map_execute_commands;

  std::list<std::string> m_attr_adis16505;
  std::list<std::string> m_attr_adis1657x;
  std::list<std::string> m_attr_current_device;
  std::map<std::string, int32_t> m_int32_current_params;
  std::map<std::string, int64_t> m_uint32_current_params;
  std::map<std::string, double> m_double_current_params;
};

#endif  // IMU_CONTROL_PARAMETERS_H
