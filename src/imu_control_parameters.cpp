/***************************************************************************/ /**
 *   @file   imu_control_parameters.cpp
 *   @brief  Set ros parameter in libiio - implementation
 *   @author Vasile Holonec (Vasile.Holonec@analog.com)
 ********************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.

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
 *******************************************************************************/

#include "imu_ros2/imu_control_parameters.h"

#include <map>
#include <string>
#include <thread>

ImuControlParameters::ImuControlParameters(std::shared_ptr<rclcpp::Node> & node)
{
  declareFunctions();
  init(node);
}

ImuControlParameters::~ImuControlParameters() {}

void ImuControlParameters::declareFunctions()
{
  m_func_map_update_int32_params["accel_calibbias_x"] = &IIOWrapper::update_accel_calibbias_x;
  m_func_map_update_int32_params["accel_calibbias_y"] = &IIOWrapper::update_accel_calibbias_y;
  m_func_map_update_int32_params["accel_calibbias_z"] = &IIOWrapper::update_accel_calibbias_z;
  m_func_map_update_int32_params["anglvel_calibbias_x"] = &IIOWrapper::update_anglvel_calibbias_x;
  m_func_map_update_int32_params["anglvel_calibbias_y"] = &IIOWrapper::update_anglvel_calibbias_y;
  m_func_map_update_int32_params["anglvel_calibbias_z"] = &IIOWrapper::update_anglvel_calibbias_z;

  m_func_map_update_uint32_params["filter_size"] = &IIOWrapper::update_filter_size;
  m_func_map_update_uint32_params["internal_sensor_bandwidth"] =
    &IIOWrapper::update_internal_sensor_bandwidth;
  m_func_map_update_uint32_params["point_of_percussion_alignment"] =
    &IIOWrapper::update_point_of_percussion_alignment;
  m_func_map_update_uint32_params["linear_acceleration_compensation"] =
    &IIOWrapper::update_linear_acceleration_compensation;
  m_func_map_update_uint32_params["burst_size_selection"] =
    &IIOWrapper::update_burst_size_selection;
  m_func_map_update_uint32_params["timestamp32"] = &IIOWrapper::update_timestamp32;
  m_func_map_update_uint32_params["internal_sync_enable_4khz"] =
    &IIOWrapper::update_internal_sync_enable_4khz;

  m_func_map_update_uint32_params["bias_correction_time_base_control"] =
    &IIOWrapper::update_bias_correction_time_base_control;
  m_func_map_update_uint32_params["x_axis_gyroscope_bias_correction_enable"] =
    &IIOWrapper::update_x_axis_gyroscope_bias_correction_enable;
  m_func_map_update_uint32_params["y_axis_gyroscope_bias_correction_enable"] =
    &IIOWrapper::update_y_axis_gyroscope_bias_correction_enable;
  m_func_map_update_uint32_params["z_axis_gyroscope_bias_correction_enable"] =
    &IIOWrapper::update_z_axis_gyroscope_bias_correction_enable;
  m_func_map_update_uint32_params["x_axis_accelerometer_bias_correction_enable"] =
    &IIOWrapper::update_x_axis_accelerometer_bias_correction_enable;
  m_func_map_update_uint32_params["y_axis_accelerometer_bias_correction_enable"] =
    &IIOWrapper::update_y_axis_accelerometer_bias_correction_enable;
  m_func_map_update_uint32_params["z_axis_accelerometer_bias_correction_enable"] =
    &IIOWrapper::update_z_axis_accelerometer_bias_correction_enable;

  m_func_map_get_int32_params["accel_calibbias_x"] = &IIOWrapper::accel_x_calibbias;
  m_func_map_get_int32_params["accel_calibbias_y"] = &IIOWrapper::accel_y_calibbias;
  m_func_map_get_int32_params["accel_calibbias_z"] = &IIOWrapper::accel_z_calibbias;
  m_func_map_get_int32_params["anglvel_calibbias_x"] = &IIOWrapper::anglvel_x_calibbias;
  m_func_map_get_int32_params["anglvel_calibbias_y"] = &IIOWrapper::anglvel_y_calibbias;
  m_func_map_get_int32_params["anglvel_calibbias_z"] = &IIOWrapper::anglvel_z_calibbias;

  m_func_map_get_uint32_params["filter_size"] = &IIOWrapper::filter_size;
  m_func_map_get_uint32_params["internal_sensor_bandwidth"] =
    &IIOWrapper::internal_sensor_bandwidth;
  m_func_map_get_uint32_params["point_of_percussion_alignment"] =
    &IIOWrapper::point_of_percussion_alignment;
  m_func_map_get_uint32_params["linear_acceleration_compensation"] =
    &IIOWrapper::linear_acceleration_compensation;
  m_func_map_get_uint32_params["burst_size_selection"] = &IIOWrapper::burst_size_selection;
  m_func_map_get_uint32_params["timestamp32"] = &IIOWrapper::timestamp32;
  m_func_map_get_uint32_params["internal_sync_enable_4khz"] =
    &IIOWrapper::internal_sync_enable_4khz;

  m_func_map_get_uint32_params["bias_correction_time_base_control"] =
    &IIOWrapper::bias_correction_time_base_control;
  m_func_map_get_uint32_params["x_axis_gyroscope_bias_correction_enable"] =
    &IIOWrapper::x_axis_gyroscope_bias_correction_enable;
  m_func_map_get_uint32_params["y_axis_gyroscope_bias_correction_enable"] =
    &IIOWrapper::y_axis_gyroscope_bias_correction_enable;
  m_func_map_get_uint32_params["z_axis_gyroscope_bias_correction_enable"] =
    &IIOWrapper::z_axis_gyroscope_bias_correction_enable;
  m_func_map_get_uint32_params["x_axis_accelerometer_bias_correction_enable"] =
    &IIOWrapper::x_axis_accelerometer_bias_correction_enable;
  m_func_map_get_uint32_params["y_axis_accelerometer_bias_correction_enable"] =
    &IIOWrapper::y_axis_accelerometer_bias_correction_enable;
  m_func_map_get_uint32_params["z_axis_accelerometer_bias_correction_enable"] =
    &IIOWrapper::z_axis_accelerometer_bias_correction_enable;

  m_func_map_get_double_params["sampling_frequency"] = &IIOWrapper::sampling_frequency;
  m_func_map_update_double_params["sampling_frequency"] = &IIOWrapper::update_sampling_frequency;

  m_func_map_execute_commands["software_reset"] = &IIOWrapper::software_reset;
  m_func_map_execute_commands["flash_memory_test"] = &IIOWrapper::flash_memory_test;
  m_func_map_execute_commands["flash_memory_update"] = &IIOWrapper::flash_memory_update;
  m_func_map_execute_commands["sensor_self_test"] = &IIOWrapper::sensor_self_test;
  m_func_map_execute_commands["factory_calibration_restore"] =
    &IIOWrapper::factory_calibration_restore;

  // declare atributes for adis16505
  m_attr_adis16505.push_back("anglvel_calibbias_x");
  m_attr_adis16505.push_back("anglvel_calibbias_y");
  m_attr_adis16505.push_back("anglvel_calibbias_z");
  m_attr_adis16505.push_back("accel_calibbias_x");
  m_attr_adis16505.push_back("accel_calibbias_y");
  m_attr_adis16505.push_back("accel_calibbias_z");

  m_attr_adis16505.push_back("filter_size");
  m_attr_adis16505.push_back("internal_sensor_bandwidth");
  m_attr_adis16505.push_back("point_of_percussion_alignment");
  m_attr_adis16505.push_back("linear_acceleration_compensation");
  m_attr_adis16505.push_back("burst_size_selection");
  m_attr_adis16505.push_back("sampling_frequency");

  // declare atributes for adis1657x
  m_attr_adis1657x.push_back("anglvel_calibbias_x");
  m_attr_adis1657x.push_back("anglvel_calibbias_y");
  m_attr_adis1657x.push_back("anglvel_calibbias_z");
  m_attr_adis1657x.push_back("accel_calibbias_x");
  m_attr_adis1657x.push_back("accel_calibbias_y");
  m_attr_adis1657x.push_back("accel_calibbias_z");

  m_attr_adis1657x.push_back("filter_size");
  m_attr_adis1657x.push_back("internal_sensor_bandwidth");
  m_attr_adis1657x.push_back("point_of_percussion_alignment");
  m_attr_adis1657x.push_back("linear_acceleration_compensation");
  m_attr_adis1657x.push_back("burst_size_selection");
  m_attr_adis1657x.push_back("timestamp32");
  m_attr_adis1657x.push_back("internal_sync_enable_4khz");

  m_attr_adis1657x.push_back("bias_correction_time_base_control");
  m_attr_adis1657x.push_back("x_axis_gyroscope_bias_correction_enable");
  m_attr_adis1657x.push_back("y_axis_accelerometer_bias_correction_enable");
  m_attr_adis1657x.push_back("z_axis_accelerometer_bias_correction_enable");
  m_attr_adis1657x.push_back("x_axis_accelerometer_bias_correction_enable");
  m_attr_adis1657x.push_back("y_axis_gyroscope_bias_correction_enable");
  m_attr_adis1657x.push_back("z_axis_gyroscope_bias_correction_enable");
  m_attr_adis1657x.push_back("sampling_frequency");

  switch (IIOWrapper::s_device_name_enum) {
    case IIODeviceName::ADIS16505:
      m_attr_current_device = m_attr_adis16505;
      break;
    case IIODeviceName::ADIS1657X:
      m_attr_current_device = m_attr_adis1657x;
      m_func_map_execute_commands["bias_correction_update"] = &IIOWrapper::bias_correction_update;
      break;
    default: {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp_device_error"), "Device is not supported");
      break;
    }
  }
}

void ImuControlParameters::init(std::shared_ptr<rclcpp::Node> & node)
{
  m_node = node;
  uint32_t u32Param;
  int64_t i64Param;
  int32_t i32Param;
  double dParam;

  for (const std::string & key : m_attr_current_device) {
    if (m_func_map_get_int32_params.find(key) != m_func_map_get_int32_params.end()) {
      (m_iio_wrapper.*(m_func_map_get_int32_params[key]))(i32Param);
      m_node->declare_parameter(key, i32Param);
      m_int32_current_params[key] = i32Param;
    } else if (m_func_map_get_uint32_params.find(key) != m_func_map_get_uint32_params.end()) {
      (m_iio_wrapper.*(m_func_map_get_uint32_params[key]))(u32Param);
      i64Param = u32Param;
      m_node->declare_parameter(key, i64Param);
      m_uint32_current_params[key] = i64Param;
    } else if (m_func_map_get_double_params.find(key) != m_func_map_get_double_params.end()) {
      (m_iio_wrapper.*(m_func_map_get_double_params[key]))(&dParam);
      m_node->declare_parameter(key, dParam);
      m_double_current_params[key] = dParam;
    } else {
      // do nothing
    }
  }

  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description =
    "command_to_execute values:\n \
    software_reset: performs a software reset on the device \n \
    flash_memory_test: performs a flash memory test on the device \n \
    flash_memory_update: performs a flash memory update on the device \n \
    sensor_self_test: performs a sensor self test on the device \n \
    factory_calibration_restore: performs a factory calibration restore on the device";

  switch (IIOWrapper::s_device_name_enum) {
    case IIODeviceName::ADIS16505:

      break;
    case IIODeviceName::ADIS1657X:
      param_desc.description.append(
        "\n \
    bias_correction_update: triggers a bias correction, using the bias correction factors");
      m_func_map_execute_commands["bias_correction_update"] = &IIOWrapper::bias_correction_update;
      break;
    default: {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp_device_error"), "Device is not supported");
      break;
    }
  }

  m_node->declare_parameter("command_to_execute", "no_command", param_desc);
}

void ImuControlParameters::updateRosParams()
{
  double doubleDriverParam;
  double doubleCurrentParam;
  int32_t int32DriverParam;
  int32_t int32CurrentParam;

  uint32_t u32DriverParam;
  int64_t int64DriverParam;
  int64_t int64CurrentParam;

  for (const std::string & key : m_attr_current_device) {
    const char * ckey = key.c_str();
    if (m_func_map_get_double_params.find(key) != m_func_map_get_double_params.end()) {
      (m_iio_wrapper.*(m_func_map_get_double_params[key]))(&doubleDriverParam);
      doubleCurrentParam = m_node->get_parameter(key).get_parameter_value().get<double>();
      if (doubleDriverParam != doubleCurrentParam) {
        m_node->set_parameter(rclcpp::Parameter(ckey, doubleDriverParam));
        RCLCPP_INFO(
          rclcpp::get_logger("rclcpp_imucontrolparameter"), "ros parameter new values %s: %f", ckey,
          doubleDriverParam);
      }
    } else if (m_func_map_get_int32_params.find(key) != m_func_map_get_int32_params.end()) {
      (m_iio_wrapper.*(m_func_map_get_int32_params[key]))(int32DriverParam);
      int32CurrentParam = m_node->get_parameter(key).get_parameter_value().get<int32_t>();
      if (int32DriverParam != int32CurrentParam) {
        m_node->set_parameter(rclcpp::Parameter(ckey, int32DriverParam));
        RCLCPP_INFO(
          rclcpp::get_logger("rclcpp_imucontrolparameter"), "ros parameter new values %s: %d", ckey,
          int32DriverParam);
      }
    } else if (m_func_map_get_uint32_params.find(key) != m_func_map_get_uint32_params.end()) {
      (m_iio_wrapper.*(m_func_map_get_uint32_params[key]))(u32DriverParam);
      int64CurrentParam = m_node->get_parameter(key).get_parameter_value().get<int64_t>();
      if (u32DriverParam != (uint32_t)int64CurrentParam) {
        int64DriverParam = u32DriverParam;
        m_node->set_parameter(rclcpp::Parameter(ckey, int64DriverParam));
        RCLCPP_INFO(
          rclcpp::get_logger("rclcpp_imucontrolparameter"), "ros parameter new values %s: %d", ckey,
          u32DriverParam);
      }
    } else {
      // do nothing
    }
  }
}

void ImuControlParameters::setParametersDouble()
{
  double requestedValue;

  for (const std::string & key : m_attr_current_device) {
    if (m_func_map_get_double_params.find(key) != m_func_map_get_double_params.end()) {
      requestedValue = m_node->get_parameter(key).get_parameter_value().get<double>();

      if (requestedValue != m_double_current_params[key]) {
        // update value in hardware
        const char * ckey = key.c_str();
        if (!(m_iio_wrapper.*(m_func_map_update_double_params[key]))(requestedValue)) {
          RCLCPP_INFO(
            rclcpp::get_logger("rclcpp_imucontrolparameter"),
            "error on set parameter %s with %f value; current value remained %f", ckey,
            m_double_current_params[key], requestedValue);
        } else {
          RCLCPP_INFO(
            rclcpp::get_logger("rclcpp_imucontrolparameter"),
            "successfully set parameter %s: old value = %f new value = %f", ckey,
            m_double_current_params[key], requestedValue);
          m_double_current_params[key] = requestedValue;
        }
      }
    }
  }
}

void ImuControlParameters::setParametersInt32()
{
  int32_t requestedValue;

  for (const std::string & key : m_attr_current_device) {
    if (m_func_map_get_int32_params.find(key) != m_func_map_get_int32_params.end()) {
      requestedValue = m_node->get_parameter(key).get_parameter_value().get<int32_t>();

      if (requestedValue != m_int32_current_params[key]) {
        // update value in hardware
        const char * ckey = key.c_str();
        if (!(m_iio_wrapper.*(m_func_map_update_int32_params[key]))(requestedValue)) {
          RCLCPP_INFO(
            rclcpp::get_logger("rclcpp_imucontrolparameter"),
            "error on set parameter %s with %d value; current value remained %d", ckey,
            m_int32_current_params[key], requestedValue);
        } else {
          RCLCPP_INFO(
            rclcpp::get_logger("rclcpp_imucontrolparameter"),
            "successfully set parameter %s: old value = %d new value = %d", ckey,
            m_int32_current_params[key], requestedValue);
          m_int32_current_params[key] = requestedValue;
        }
      }
    }
  }
}

void ImuControlParameters::setParametersUint32()
{
  int64_t requestedValue;

  for (const std::string & key : m_attr_current_device) {
    if (m_func_map_get_uint32_params.find(key) != m_func_map_get_uint32_params.end()) {
      requestedValue = m_node->get_parameter(key).get_parameter_value().get<int64_t>();

      if (requestedValue != m_uint32_current_params[key]) {
        // update value in hardware
        const char * ckey = key.c_str();
        if (!(m_iio_wrapper.*(m_func_map_update_uint32_params[key]))(requestedValue)) {
          RCLCPP_INFO(
            rclcpp::get_logger("rclcpp_imucontrolparameter"),
            "error on set parameter %s with %d value; current value remained %d", ckey,
            m_uint32_current_params[key], requestedValue);
        } else {
          RCLCPP_INFO(
            rclcpp::get_logger("rclcpp_imucontrolparameter"),
            "successfully set parameter %s: old value = %d new value = %d", ckey,
            m_uint32_current_params[key], requestedValue);
          m_uint32_current_params[key] = requestedValue;
        }
      }
    }
  }
}

void ImuControlParameters::handleCommand()
{
  std::string requestedCommand =
    m_node->get_parameter("command_to_execute").get_parameter_value().get<std::string>();

  if (m_func_map_execute_commands.find(requestedCommand) != m_func_map_execute_commands.end()) {
    if (!(m_iio_wrapper.*(m_func_map_execute_commands[requestedCommand]))()) {
      RCLCPP_INFO(
        rclcpp::get_logger("rclcpp_imucontrolparameter"), "error on executing command %s",
        requestedCommand.c_str());
    } else {
      RCLCPP_INFO(
        rclcpp::get_logger("rclcpp_imucontrolparameter"), "executed command %s",
        requestedCommand.c_str());
      // update the rest of parameters based on new changes in hardware
      updateRosParams();
      m_node->set_parameter(rclcpp::Parameter("command_to_execute", "no_command"));
    }
  }
}

void ImuControlParameters::run()
{
  std::thread::id this_id = std::this_thread::get_id();
  std::cout << "thread parameter " << this_id << " started...\n";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp_imucontrolparameter"), "startThread: '%d'", this_id);

  while (rclcpp::ok()) {
    setParametersInt32();
    setParametersUint32();
    setParametersDouble();
    handleCommand();
    rclcpp::spin_some(m_node);
  }

  this_id = std::this_thread::get_id();
  std::cout << "thread parameter " << this_id << " ended...\n";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp_imucontrolparameter"), "endThread: '%d'", this_id);
}
