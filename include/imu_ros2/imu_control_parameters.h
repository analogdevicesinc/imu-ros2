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

#include "imu_ros2/iio_wrapper.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

/**
 * @brief Class for handling device parameters.
 */
class ImuControlParameters
{
public:
  /**
   * @brief Constructor for ImuControlParameters.
   * @param node The ros2 Node instance.
   */
  ImuControlParameters(std::shared_ptr<rclcpp::Node> & node);

  /**
   * @brief Destructor for ImuControlParameters.
   */
  ~ImuControlParameters();

  /**
   * @brief Handle ROS parameters and perform the necessary updated in the
   * hardware.
   */
  void handleControlParams();

private:
  /**
   * @brief Add the list of adis parameters to m_attr_current_device.
   */
  void declareAdisAttributes();

  /**
   * @brief Add the list of IIO update functions to
   * m_func_map_update_int32_params map. This map contains the update APIs for
   * each int32 type parameter.
   */
  void mapIIOUpdateFunctionsInt32();

  /**
   * @brief Add the list of IIO read functions to
   * m_func_map_get_int32_params map. This map contains the read APIs for
   * each int32 type parameter.
   */
  void mapIIOGetFunctionsInt32();

  /**
   * @brief Add the list of IIO update functions to
   * m_func_map_update_uint32_params map. This map contains the update APIs for
   * each uint32 type parameter.
   */
  void mapIIOUpdateFunctionsUint32();

  /**
   * @brief Add the list of IIO read functions to
   * m_func_map_get_uint32_params map. This map contains the read APIs for
   * each uint32 type parameter.
   */
  void mapIIOGetFunctionsUint32();

  /**
   * @brief Add the list of IIO update functions to
   * m_func_map_update_double_params map. This map contains the update APIs for
   * each double type parameter.
   */
  void mapIIOUpdateFunctionsDouble();

  /**
   * @brief Add the list of IIO read functions to
   * m_func_map_get_double_params map. This map contains the read APIs for
   * each double type parameter.
   */
  void mapIIOGetFunctionsDouble();

  /**
   * @brief Add the list of IIO command functions to
   * m_func_map_execute_commands map. This map contains the command APIs for
   * which can be triggered in the hardware.
   */
  void mapIIOCommandFunctions();

  /**
   * @brief Add a parameter description for each parameter in
   * m_param_description.
   */
  void declareParameterDescription();

  /**
   * @brief Declare the ros parameters and initialize their values with values
   * read from hardware.
   */
  void declareParameters();

  /**
   * @brief Update all ros parameter values by reading them from hardware.
   */
  void updateParamsFromHardware();

  /**
   * @brief Handles a parameter change of type double. Performs the necessary
   * updates in the hardware and logs the result.
   */
  void handleDoubleParamChange();

  /**
   * @brief Handles a parameter change of type int32. Performs the necessary
   * updates in the hardware and logs the result.
   */
  void handleInt32ParamChange();

  /**
   * @brief Handles a parameter change of type uint32. Performs the necessary
   * updates in the hardware and logs the result.
   */
  void handleUint32ParamChange();

  /**
   * @brief Handles a command. Trigger the command in the hardware if necessary
   * and logs the result.
   */
  void handleCommands();

  /*! The ros2 Node data member */
  std::shared_ptr<rclcpp::Node> m_node;

  /*! This data member is used to access sensor information via libiio. */
  IIOWrapper m_iio_wrapper;

  /*! This variable contains the current value for the command to be executed. */
  std::string m_command_to_execute;

  /*! Declare function type for parameter updating APIs of type uint32. */
  typedef bool (IIOWrapper::*UpdateUint32Params)(uint32_t);

  /*! Declare map type for parameter updating APIs of type uint32. */
  typedef std::map<std::string, UpdateUint32Params> UpdateUint32ParamsMapType;

  /*! Update parameter map which contains the update function call for each parameter of type uint32. */
  UpdateUint32ParamsMapType m_func_map_update_uint32_params;

  /*! Declare function type for parameter reading APIs of type uint32. */
  typedef bool (IIOWrapper::*GetUint32Params)(uint32_t &);

  /*! Declare map type for parameter reading APIs of type uint32. */
  typedef std::map<std::string, GetUint32Params> GetUint32ParamsMapType;

  /*! Read parameter map which contains the read function call for each parameter of type uint32. */
  GetUint32ParamsMapType m_func_map_get_uint32_params;

  /*! Declare function type for parameter updating APIs of type int32. */
  typedef bool (IIOWrapper::*UpdateInt32Params)(int32_t);

  /*! Declare map type for parameter updating APIs of type int32. */
  typedef std::map<std::string, UpdateInt32Params> UpdateInt32ParamsMapType;

  /*! Update parameter map which contains the update function call for each parameter of type int32. */
  UpdateInt32ParamsMapType m_func_map_update_int32_params;

  /*! Declare function type for parameter reading APIs of type int32. */
  typedef bool (IIOWrapper::*GetInt32Params)(int32_t &);

  /*! Declare map type for parameter reading APIs of type int32. */
  typedef std::map<std::string, GetInt32Params> GetInt32ParamsMapType;

  /*! Read parameter map which contains the read function call for each parameter of type int32. */
  GetInt32ParamsMapType m_func_map_get_int32_params;

  /*! Declare function type for parameter updating APIs of type double. */
  typedef bool (IIOWrapper::*UpdateDoubleParams)(double);

  /*! Declare map type for parameter updating APIs of type double. */
  typedef std::map<std::string, UpdateDoubleParams> UpdateDoubleParamsMapType;

  /*! Update parameter map which contains the update function call for each parameter of type double. */
  UpdateDoubleParamsMapType m_func_map_update_double_params;

  /*! Declare function type for parameter reading APIs of type double. */
  typedef bool (IIOWrapper::*GetDoubleParams)(double *);

  /*! Declare map type for parameter reading APIs of type double. */
  typedef std::map<std::string, GetDoubleParams> GetDoubleParamsMapType;

  /*! Read parameter map which contains the read function call for each parameter of type double. */
  GetDoubleParamsMapType m_func_map_get_double_params;

  /*! Declare function type for command execution APIs. */
  typedef bool (IIOWrapper::*ExecuteCommands)();

  /*! Declare map type for command APIs. */
  typedef std::map<std::string, ExecuteCommands> ExecuteCommandsMapType;

  /*! Command map which contains the trigger function call for each command. */
  ExecuteCommandsMapType m_func_map_execute_commands;

  /*! Attribute list which stores the current attribute names of the device. */
  std::list<std::string> m_attr_current_device;

  /*! Parameter map of type int32 which stores the current value of the parameter. */
  std::map<std::string, int32_t> m_int32_current_params;

  /*! Parameter map of type uint32 which stores the current value of the parameter. */
  std::map<std::string, int64_t> m_uint32_current_params;

  /*! Parameter map of type double which stores the current value of the parameter. */
  std::map<std::string, double> m_double_current_params;

  /*! Parameter map with parameter description. */
  std::map<std::string, std::string> m_param_description;

  /*! Parameter map with IntegerRange for parameter constraints. */
  std::map<std::string, rcl_interfaces::msg::IntegerRange> m_param_constraints_integer;

  /*! Parameter map with FloatingPointRange for parameter constraints. */
  std::map<std::string, rcl_interfaces::msg::FloatingPointRange> m_param_constraints_floating;
};

#endif  // IMU_CONTROL_PARAMETERS_H
