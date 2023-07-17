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

/**
 * \brief Class for setting libiio parameters from ros2 paramenters.
 *
 * This class modifies libiio parameters with ros2 parameters
 * read from console.
 */
class ImuControlParameters : public RosTask
{
public:

  /**
   * \brief Constructor for ImuControlParameters.
   *
   * This is the default constructor for class
   *  ImuControlParameters.
   *
   * @param node The ros2 Node instance.
   */
  ImuControlParameters(std::shared_ptr<rclcpp::Node> & node);

  /**
   * \brief Destructor for ImuControlParameters.
   *
   * This is a destructor for ImuControlParameters.
   *
   */
  ~ImuControlParameters();

  /**
   * @brief Declare iio wrapper functions
   *
   * This function create a map for accesing the functions from iio wrapper.
   *
   */
  void declareFunctions();

  /**
   * @brief Initialize class with ros2 Node instance.
   *
   * This function initialize the the class that inherit
   * this interface wiht a ros2 Node instance.
   *
   * @param node The ros2 Node instance.
   */
  void init(std::shared_ptr<rclcpp::Node> & node);

  /**
   * @brief Set parameters that have double value.
   *
   * This function set parameters in libiio that have double value.
   *
   */
  void setParametersDouble();

  /**
   * @brief Set parameters that have int32 value.
   *
   * This function set parameters in libiio that have int32 value.
   *
   */
  void setParametersInt32();

  /**
   * @brief Set parameters that have uint32 value.
   *
   * This function set parameters in libiio that have uint32 value.
   *
   */
  void setParametersUint32();

  /**
   * @brief Trigger libiio command.
   *
   * This function trigger libiio commands when ros2 parameters is set.
   *
   */
  void handleCommand();

  /**
   * @brief Update ros parameters.
   *
   * This function update ros parameters with new values from libiio.
   *
   */
  void updateRosParams();

  /**
   * @brief Read driver parameters.
   *
   * This function read driver parameters.
   *
   */
  void readDriverParams();

  /**
   * @brief Read ros2 parameters and update with values in libiio.
   *
   * This function read ros2 parameters and update with values in libiio
   *
   */
  void run() override;

private:
  std::shared_ptr<rclcpp::Node> m_node; /**< The ros2 Node data member */
  IIOWrapper m_iio_wrapper; /**< This data member access information from libiio */

  std::string m_command_to_execute; /**< This variable contain command to execute in libiio */

  typedef bool (IIOWrapper::*UpdateUint32Params)(uint32_t); /**< Declare function with uint32_t parameter and return bool */
  typedef std::map<std::string, UpdateUint32Params> UpdateUint32ParamsMapType; /**< Declare a map with UpdateUint32Params functions */
  UpdateUint32ParamsMapType m_func_map_update_uint32_params; /**< Declare a variable of UpdateUint32ParamsMapType */

  typedef bool (IIOWrapper::*GetUint32Params)(uint32_t &); /**< Declare function with uint32_t& parameter and return bool */
  typedef std::map<std::string, GetUint32Params> GetUint32ParamsMapType; /**< Declare a map with GetUint32Params functions */
  GetUint32ParamsMapType m_func_map_get_uint32_params; /**< Declare a variable of GetUint32ParamsMapType */

  typedef bool (IIOWrapper::*UpdateInt32Params)(int32_t); /**< Declare function with int32_t parameter and return bool */
  typedef std::map<std::string, UpdateInt32Params> UpdateInt32ParamsMapType; /**< Declare a map with UpdateInt32Params functions */
  UpdateInt32ParamsMapType m_func_map_update_int32_params; /**< Declare a variable of UpdateInt32ParamsMapType */

  typedef bool (IIOWrapper::*GetInt32Params)(int32_t &); /**< Declare function with int32_t& parameter and return bool */
  typedef std::map<std::string, GetInt32Params> GetInt32ParamsMapType; /**< Declare a map with GetInt32Params functions */
  GetInt32ParamsMapType m_func_map_get_int32_params;  /**< Declare a variable of GetInt32ParamsMapType */

  typedef bool (IIOWrapper::*UpdateDoubleParams)(double); /**< Declare function with double parameter and return bool */
  typedef std::map<std::string, UpdateDoubleParams> UpdateDoubleParamsMapType; /**< Declare a map with UpdateDoubleParams functions */
  UpdateDoubleParamsMapType m_func_map_update_double_params; /**< Declare a variable of UpdateDoubleParamsMapType */

  typedef bool (IIOWrapper::*GetDoubleParams)(double *); /**< Declare function with double parameter and return bool */
  typedef std::map<std::string, GetDoubleParams> GetDoubleParamsMapType; /**< Declare a map with GetDoubleParams functions */
  GetDoubleParamsMapType m_func_map_get_double_params; /**< Declare a variable of GetDoubleParamsMapType */

  typedef bool (IIOWrapper::*ExecuteCommands)(); /**< Declare function that return bool */
  typedef std::map<std::string, ExecuteCommands> ExecuteCommandsMapType; /**< Declare a map with ExecuteCommands functions */
  ExecuteCommandsMapType m_func_map_execute_commands; /**< Declare a variable of ExecuteCommandsMapType */

  std::list<std::string> m_attr_adis1650x; /**< Declare a variable with adis1650x attributes */
  std::list<std::string> m_attr_adis1657x; /**< Declare a variable with adis1657x attributes */
  std::list<std::string> m_attr_current_device; /**< Declare a variable current attributes, adis1650x or adis1657x */
  std::map<std::string, int32_t> m_int32_current_params; /**< Declare a variable current int32_t parameters, adis1650x or adis1657x */
  std::map<std::string, int64_t> m_uint32_current_params; /**< Declare a variable current int64_t parameters, adis1650x or adis1657x */
  std::map<std::string, double> m_double_current_params;  /**< Declare a variable current double parameters, adis1650x or adis1657x */
};

#endif  // IMU_CONTROL_PARAMETERS_H
