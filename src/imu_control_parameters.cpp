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

#include "adi_imu/imu_control_parameters.h"

#include <inttypes.h>

#include <map>
#include <string>
#include <thread>

ImuControlParameters::ImuControlParameters(std::shared_ptr<rclcpp::Node> & node)
{
  m_node = node;
  declareAdisAttributes();
  mapIIOUpdateFunctionsInt32();
  mapIIOGetFunctionsInt32();
  mapIIOUpdateFunctionsUint32();
  mapIIOGetFunctionsUint32();
  mapIIOUpdateFunctionsDouble();
  mapIIOGetFunctionsDouble();
  mapIIOCommandFunctions();
  declareParameterDescription();
  declareParameters();
}

ImuControlParameters::~ImuControlParameters() {}

void ImuControlParameters::declareAdisAttributes()
{
  m_attr_current_device.push_back("anglvel_calibbias_x");
  m_attr_current_device.push_back("anglvel_calibbias_y");
  m_attr_current_device.push_back("anglvel_calibbias_z");
  m_attr_current_device.push_back("accel_calibbias_x");
  m_attr_current_device.push_back("accel_calibbias_y");
  m_attr_current_device.push_back("accel_calibbias_z");

#ifdef ADIS_HAS_CALIB_SCALE
  m_attr_current_device.push_back("anglvel_calibscale_x");
  m_attr_current_device.push_back("anglvel_calibscale_y");
  m_attr_current_device.push_back("anglvel_calibscale_z");
  m_attr_current_device.push_back("accel_calibscale_x");
  m_attr_current_device.push_back("accel_calibscale_y");
  m_attr_current_device.push_back("accel_calibscale_z");
#endif
#ifdef ADIS_SENS_BW
  m_attr_current_device.push_back("internal_sensor_bandwidth");
#endif
#ifdef ADIS_PT_OF_PERC_REG_ADDR
  m_attr_current_device.push_back("point_of_percussion_alignment");
#endif
#ifdef ADIS_MSC_CTRL_ADDR
  m_attr_current_device.push_back("linear_acceleration_compensation");
#endif
#ifdef ADIS_NULL_CNFG_ADDR
  m_attr_current_device.push_back("bias_correction_time_base_control");
  m_attr_current_device.push_back("x_axis_gyroscope_bias_correction_enable");
  m_attr_current_device.push_back("y_axis_accelerometer_bias_correction_enable");
  m_attr_current_device.push_back("z_axis_accelerometer_bias_correction_enable");
  m_attr_current_device.push_back("x_axis_accelerometer_bias_correction_enable");
  m_attr_current_device.push_back("y_axis_gyroscope_bias_correction_enable");
  m_attr_current_device.push_back("z_axis_gyroscope_bias_correction_enable");
#endif
#if defined(ADIS1654X) || defined(ADIS1655X)
  m_attr_current_device.push_back("angvel_x_filter_low_pass_3db");
  m_attr_current_device.push_back("angvel_y_filter_low_pass_3db");
  m_attr_current_device.push_back("angvel_z_filter_low_pass_3db");
  m_attr_current_device.push_back("accel_x_filter_low_pass_3db");
  m_attr_current_device.push_back("accel_y_filter_low_pass_3db");
  m_attr_current_device.push_back("accel_z_filter_low_pass_3db");
#else
  m_attr_current_device.push_back("filter_low_pass_3db_frequency");
#endif
  m_attr_current_device.push_back("sampling_frequency");
}

void ImuControlParameters::mapIIOUpdateFunctionsInt32()
{
  m_func_map_update_int32_params["accel_calibbias_x"] = &IIOWrapper::update_accel_calibbias_x;
  m_func_map_update_int32_params["accel_calibbias_y"] = &IIOWrapper::update_accel_calibbias_y;
  m_func_map_update_int32_params["accel_calibbias_z"] = &IIOWrapper::update_accel_calibbias_z;
  m_func_map_update_int32_params["anglvel_calibbias_x"] = &IIOWrapper::update_anglvel_calibbias_x;
  m_func_map_update_int32_params["anglvel_calibbias_y"] = &IIOWrapper::update_anglvel_calibbias_y;
  m_func_map_update_int32_params["anglvel_calibbias_z"] = &IIOWrapper::update_anglvel_calibbias_z;

#ifdef ADIS_HAS_CALIB_SCALE
  m_func_map_update_int32_params["accel_calibscale_x"] = &IIOWrapper::update_accel_calibscale_x;
  m_func_map_update_int32_params["accel_calibscale_y"] = &IIOWrapper::update_accel_calibscale_y;
  m_func_map_update_int32_params["accel_calibscale_z"] = &IIOWrapper::update_accel_calibscale_z;
  m_func_map_update_int32_params["anglvel_calibscale_x"] = &IIOWrapper::update_anglvel_calibscale_x;
  m_func_map_update_int32_params["anglvel_calibscale_y"] = &IIOWrapper::update_anglvel_calibscale_y;
  m_func_map_update_int32_params["anglvel_calibscale_z"] = &IIOWrapper::update_anglvel_calibscale_z;
#endif
#if defined(ADIS1654X) || defined(ADIS1655X)
  m_func_map_update_uint32_params["angvel_x_filter_low_pass_3db"] =
    &IIOWrapper::update_angvel_x_filter_low_pass_3db;
  m_func_map_update_uint32_params["angvel_y_filter_low_pass_3db"] =
    &IIOWrapper::update_angvel_x_filter_low_pass_3db;
  m_func_map_update_uint32_params["angvel_z_filter_low_pass_3db"] =
    &IIOWrapper::update_angvel_x_filter_low_pass_3db;
  m_func_map_update_uint32_params["accel_x_filter_low_pass_3db"] =
    &IIOWrapper::update_angvel_x_filter_low_pass_3db;
  m_func_map_update_uint32_params["accel_y_filter_low_pass_3db"] =
    &IIOWrapper::update_angvel_x_filter_low_pass_3db;
  m_func_map_update_uint32_params["accel_z_filter_low_pass_3db"] =
    &IIOWrapper::update_angvel_x_filter_low_pass_3db;
#else
  m_func_map_update_uint32_params["filter_low_pass_3db_frequency"] =
    &IIOWrapper::update_filter_low_pass_3db_frequency;
#endif
}

void ImuControlParameters::mapIIOGetFunctionsInt32()
{
  m_func_map_get_int32_params["accel_calibbias_x"] = &IIOWrapper::accel_x_calibbias;
  m_func_map_get_int32_params["accel_calibbias_y"] = &IIOWrapper::accel_y_calibbias;
  m_func_map_get_int32_params["accel_calibbias_z"] = &IIOWrapper::accel_z_calibbias;
  m_func_map_get_int32_params["anglvel_calibbias_x"] = &IIOWrapper::anglvel_x_calibbias;
  m_func_map_get_int32_params["anglvel_calibbias_y"] = &IIOWrapper::anglvel_y_calibbias;
  m_func_map_get_int32_params["anglvel_calibbias_z"] = &IIOWrapper::anglvel_z_calibbias;

#ifdef ADIS_HAS_CALIB_SCALE
  m_func_map_get_int32_params["accel_calibscale_x"] = &IIOWrapper::accel_x_calibscale;
  m_func_map_get_int32_params["accel_calibscale_y"] = &IIOWrapper::accel_y_calibscale;
  m_func_map_get_int32_params["accel_calibscale_z"] = &IIOWrapper::accel_z_calibscale;
  m_func_map_get_int32_params["anglvel_calibscale_x"] = &IIOWrapper::anglvel_x_calibscale;
  m_func_map_get_int32_params["anglvel_calibscale_y"] = &IIOWrapper::anglvel_y_calibscale;
  m_func_map_get_int32_params["anglvel_calibscale_z"] = &IIOWrapper::anglvel_z_calibscale;
#endif
#if defined(ADIS1654X) || defined(ADIS1655X)
  m_func_map_get_uint32_params["angvel_x_filter_low_pass_3db"] =
    &IIOWrapper::angvel_x_filter_low_pass_3db;
  m_func_map_get_uint32_params["angvel_y_filter_low_pass_3db"] =
    &IIOWrapper::angvel_x_filter_low_pass_3db;
  m_func_map_get_uint32_params["angvel_z_filter_low_pass_3db"] =
    &IIOWrapper::angvel_x_filter_low_pass_3db;
  m_func_map_get_uint32_params["accel_x_filter_low_pass_3db"] =
    &IIOWrapper::angvel_x_filter_low_pass_3db;
  m_func_map_get_uint32_params["accel_y_filter_low_pass_3db"] =
    &IIOWrapper::angvel_x_filter_low_pass_3db;
  m_func_map_get_uint32_params["accel_z_filter_low_pass_3db"] =
    &IIOWrapper::angvel_x_filter_low_pass_3db;
#else
  m_func_map_get_uint32_params["filter_low_pass_3db_frequency"] =
    &IIOWrapper::filter_low_pass_3db_frequency;
#endif
}

void ImuControlParameters::mapIIOUpdateFunctionsUint32()
{
#ifdef ADIS_SENS_BW
  m_func_map_update_uint32_params["internal_sensor_bandwidth"] =
    &IIOWrapper::update_internal_sensor_bandwidth;
#endif
#ifdef ADIS_PT_OF_PERC_REG_ADDR
  m_func_map_update_uint32_params["point_of_percussion_alignment"] =
    &IIOWrapper::update_point_of_percussion_alignment;
#endif
#ifdef ADIS_MSC_CTRL_ADDR
  m_func_map_update_uint32_params["linear_acceleration_compensation"] =
    &IIOWrapper::update_linear_acceleration_compensation;
#endif
#ifdef ADIS_NULL_CNFG_ADDR
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
#endif
}

void ImuControlParameters::mapIIOGetFunctionsUint32()
{
#ifdef ADIS_SENS_BW
  m_func_map_get_uint32_params["internal_sensor_bandwidth"] =
    &IIOWrapper::internal_sensor_bandwidth;
#endif
#ifdef ADIS_PT_OF_PERC_REG_ADDR
  m_func_map_get_uint32_params["point_of_percussion_alignment"] =
    &IIOWrapper::point_of_percussion_alignment;
#endif
#ifdef ADIS_MSC_CTRL_ADDR
  m_func_map_get_uint32_params["linear_acceleration_compensation"] =
    &IIOWrapper::linear_acceleration_compensation;
#endif
#ifdef ADIS_NULL_CNFG_ADDR
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
#endif
}

void ImuControlParameters::mapIIOUpdateFunctionsDouble()
{
  m_func_map_update_double_params["sampling_frequency"] = &IIOWrapper::update_sampling_frequency;
}

void ImuControlParameters::mapIIOGetFunctionsDouble()
{
  m_func_map_get_double_params["sampling_frequency"] = &IIOWrapper::sampling_frequency;
}

void ImuControlParameters::mapIIOCommandFunctions()
{
  m_func_map_execute_commands["software_reset"] = &IIOWrapper::software_reset;
#ifdef ADIS_FLASH_MEMORY_TEST
  m_func_map_execute_commands["flash_memory_test"] = &IIOWrapper::flash_memory_test;
#endif
  m_func_map_execute_commands["flash_memory_update"] = &IIOWrapper::flash_memory_update;
  m_func_map_execute_commands["sensor_self_test"] = &IIOWrapper::sensor_self_test;
  m_func_map_execute_commands["factory_calibration_restore"] =
    &IIOWrapper::factory_calibration_restore;
#ifdef ADIS_BIAS_CORRECTION_UPDATE
  m_func_map_execute_commands["bias_correction_update"] = &IIOWrapper::bias_correction_update;
#endif
}

void ImuControlParameters::declareParameterDescription()
{
  auto param_range_calibbias = rcl_interfaces::msg::IntegerRange{};
  param_range_calibbias.from_value = -2147483648;
  param_range_calibbias.to_value = 2147483647;
  param_range_calibbias.step = 1;

#ifndef ADIS1654X
  auto param_range_0_720 = rcl_interfaces::msg::IntegerRange{};
  param_range_0_720.from_value = 0;
  param_range_0_720.to_value = 720;
#else
  auto param_range_100_300 = rcl_interfaces::msg::IntegerRange{};
  param_range_100_300.from_value = 0;
  param_range_100_300.to_value = 300;
#endif

  auto param_range_01 = rcl_interfaces::msg::IntegerRange{};
  param_range_01.from_value = 0;
  param_range_01.to_value = 1;
  param_range_01.step = 1;

  auto param_range_03 = rcl_interfaces::msg::IntegerRange{};
#ifdef ADIS_HAS_DELTA_BURST
  param_range_03.from_value = 0;
#else
  param_range_03.from_value = 1;
#endif
  param_range_03.to_value = 3;
  param_range_03.step = 1;

  m_param_description["anglvel_calibbias_x"] = "x-axis angular velocity offset correction";
  m_param_constraints_integer["anglvel_calibbias_x"] = param_range_calibbias;

  m_param_description["anglvel_calibbias_y"] = "y-axis angular velocity offset correction";
  m_param_constraints_integer["anglvel_calibbias_y"] = param_range_calibbias;

  m_param_description["anglvel_calibbias_z"] = "z-axis angular velocity offset correction";
  m_param_constraints_integer["anglvel_calibbias_z"] = param_range_calibbias;

  m_param_description["accel_calibbias_x"] = "x-axis acceleration offset correction";
  m_param_constraints_integer["accel_calibbias_x"] = param_range_calibbias;

  m_param_description["accel_calibbias_y"] = "y-axis acceleration offset correction";
  m_param_constraints_integer["accel_calibbias_y"] = param_range_calibbias;

  m_param_description["accel_calibbias_z"] = "z-axis acceleration offset correction";
  m_param_constraints_integer["accel_calibbias_z"] = param_range_calibbias;

#ifdef ADIS1654X
  m_param_description["angvel_x_filter_low_pass_3db"] = "X angular velocity low pass 3db frequency";
  m_param_constraints_integer["angvel_x_filter_low_pass_3db"] = param_range_100_300;

  m_param_description["angvel_y_filter_low_pass_3db"] = "Y angular velocity low pass 3db frequency";
  m_param_constraints_integer["angvel_y_filter_low_pass_3db"] = param_range_100_300;

  m_param_description["angvel_z_filter_low_pass_3db"] = "Z angular velocity low pass 3db frequency";
  m_param_constraints_integer["angvel_z_filter_low_pass_3db"] = param_range_100_300;

  m_param_description["accel_x_filter_low_pass_3db"] = "X acceleration low pass 3db frequency";
  m_param_constraints_integer["accel_x_filter_low_pass_3db"] = param_range_100_300;

  m_param_description["accel_y_filter_low_pass_3db"] = "Y acceleration low pass 3db frequency";
  m_param_constraints_integer["accel_y_filter_low_pass_3db"] = param_range_100_300;

  m_param_description["accel_z_filter_low_pass_3db"] = "Z acceleration low pass 3db frequency";
  m_param_constraints_integer["accel_z_filter_low_pass_3db"] = param_range_100_300;
#else
  m_param_description["filter_low_pass_3db_frequency"] = "Low pass 3db frequency";
  m_param_constraints_integer["filter_low_pass_3db_frequency"] = param_range_0_720;
#endif

#ifdef ADIS_HAS_CALIB_SCALE

  m_param_description["anglvel_calibscale_x"] = "x-axis angular velocity scale correction";
  m_param_constraints_integer["anglvel_calibscale_x"] = param_range_calibbias;

  m_param_description["anglvel_calibscale_y"] = "y-axis angular velocity scale correction";
  m_param_constraints_integer["anglvel_calibscale_y"] = param_range_calibbias;

  m_param_description["anglvel_calibscale_z"] = "z-axis angular velocity scale correction";
  m_param_constraints_integer["anglvel_calibscale_z"] = param_range_calibbias;

  m_param_description["accel_calibscale_x"] = "x-axis acceleration scale correction";
  m_param_constraints_integer["accel_calibscale_x"] = param_range_calibbias;

  m_param_description["accel_calibscale_y"] = "y-axis acceleration scale correction";
  m_param_constraints_integer["accel_calibscale_y"] = param_range_calibbias;

  m_param_description["accel_calibscale_z"] = "z-axis acceleration scale correction";
  m_param_constraints_integer["accel_calibscale_z"] = param_range_calibbias;

#endif

#ifdef ADIS_SENS_BW
  m_param_description["internal_sensor_bandwidth"] =
    "\n0: wide bandwidth"
    "\n1: 370 Hz";
  m_param_constraints_integer["internal_sensor_bandwidth"] = param_range_01;
#endif
#ifdef ADIS_PT_OF_PERC_REG_ADDR
  m_param_description["point_of_percussion_alignment"] =
    "\n0: point of percussion alignment disable"
    "\n1: point of percussion alignment enable";
  m_param_constraints_integer["point_of_percussion_alignment"] = param_range_01;
#endif
#ifdef ADIS_MSC_CTRL_ADDR
  m_param_description["linear_acceleration_compensation"] =
    "\n0: linear acceleration compensation disable"
    "\n1: linear acceleration compensation enable";
  m_param_constraints_integer["linear_acceleration_compensation"] = param_range_01;
#endif
#ifdef ADIS_NULL_CNFG_ADDR
  auto param_range_0_12 = rcl_interfaces::msg::IntegerRange{};
  param_range_0_12.from_value = 0;
  param_range_0_12.to_value = 12;
  param_range_0_12.step = 1;

  m_param_description["bias_correction_time_base_control"] = "Time base control";
  m_param_constraints_integer["bias_correction_time_base_control"] = param_range_0_12;

  m_param_description["x_axis_gyroscope_bias_correction_enable"] =
    "\n0: x-axis gyroscope bias correction disabled"
    "\n1: x-axis gyroscope bias correction enabled";
  m_param_constraints_integer["x_axis_gyroscope_bias_correction_enable"] = param_range_01;

  m_param_description["y_axis_gyroscope_bias_correction_enable"] =
    "\n0: y-axis gyroscope bias correction disabled"
    "\n1: y-axis gyroscope bias correction enabled";
  m_param_constraints_integer["y_axis_gyroscope_bias_correction_enable"] = param_range_01;

  m_param_description["z_axis_gyroscope_bias_correction_enable"] =
    "\n0: z-axis gyroscope bias correction disabled"
    "\n1: z-axis gyroscope bias correction enabled";
  m_param_constraints_integer["z_axis_gyroscope_bias_correction_enable"] = param_range_01;

  m_param_description["x_axis_accelerometer_bias_correction_enable"] =
    "\n0: x-axis accelerometer bias correction disabled"
    "\n1: x-axis accelerometer bias correction enabled";
  m_param_constraints_integer["x_axis_accelerometer_bias_correction_enable"] = param_range_01;

  m_param_description["y_axis_accelerometer_bias_correction_enable"] =
    "\n0: y-axis accelerometer bias correction disabled"
    "\n1: z-axis accelerometer bias correction enabled";
  m_param_constraints_integer["y_axis_accelerometer_bias_correction_enable"] = param_range_01;

  m_param_description["z_axis_accelerometer_bias_correction_enable"] =
    "\n0: z-axis accelerometer bias correction disabled"
    "\n1: z-axis accelerometer bias correction enabled";
  m_param_constraints_integer["z_axis_accelerometer_bias_correction_enable"] = param_range_01;
#endif

  m_param_description["measured_data_topic_selection"] = "\nmeasured_data_topic_selection values:";
#ifdef ADIS_HAS_DELTA_BURST
  m_param_description["measured_data_topic_selection"].append(
    "\n0: measured data is published on /velangtempdata topic");
#endif
  m_param_description["measured_data_topic_selection"].append(
    "\n1: measured data is published on /accelgyrotempdata topic");

  m_param_description["measured_data_topic_selection"].append(
    "\n2: measured data is published on /imu topic"
    "\n3: measured data is published on /imufullmeasureddata topic "
    "(default)");

  m_param_constraints_integer["measured_data_topic_selection"] = param_range_03;

  auto param_range_float = rcl_interfaces::msg::FloatingPointRange{};
  param_range_float.from_value = 1.0;
  param_range_float.to_value = ADIS_MAX_SAMP_FREQ;
  m_param_description["sampling_frequency"] = "Device sampling frequency";
  m_param_constraints_floating["sampling_frequency"] = param_range_float;
}

void ImuControlParameters::declareParameters()
{
  uint32_t u32Param = 0;
  int64_t i64Param = 0;
  int32_t i32Param = 0;
  double dParam = 0;

  for (const std::string & key : m_attr_current_device) {
    if (m_func_map_get_int32_params.find(key) != m_func_map_get_int32_params.end()) {
      (m_iio_wrapper.*(m_func_map_get_int32_params[key]))(i32Param);

      auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
      param_desc.description = m_param_description[key];
      param_desc.integer_range.push_back(m_param_constraints_integer[key]);

      m_node->declare_parameter(key, i32Param, param_desc);
      m_int32_current_params[key] = i32Param;
    } else if (m_func_map_get_uint32_params.find(key) != m_func_map_get_uint32_params.end()) {
      (m_iio_wrapper.*(m_func_map_get_uint32_params[key]))(u32Param);
      i64Param = u32Param;

      auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
      param_desc.description = m_param_description[key];
      param_desc.integer_range.push_back(m_param_constraints_integer[key]);

      m_node->declare_parameter(key, i64Param, param_desc);
      m_uint32_current_params[key] = i64Param;
    } else if (m_func_map_get_double_params.find(key) != m_func_map_get_double_params.end()) {
      (m_iio_wrapper.*(m_func_map_get_double_params[key]))(&dParam);

      auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
      param_desc.description = m_param_description[key];
      param_desc.floating_point_range.push_back(m_param_constraints_floating[key]);

      m_node->declare_parameter(key, dParam, param_desc);
      m_double_current_params[key] = dParam;
    } else {
      // do nothing
    }
  }

  auto param_desc_topic = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_topic.description = m_param_description["measured_data_topic_selection"];
  param_desc_topic.integer_range.push_back(
    m_param_constraints_integer["measured_data_topic_selection"]);
  m_node->declare_parameter("measured_data_topic_selection", FULL_MEASURED_DATA, param_desc_topic);

  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description =
    "\ncommand_to_execute values:"
    "\nsoftware_reset: performs a software reset on the device"
#ifdef ADIS_FLASH_MEMORY_TEST
    "\nflash_memory_test: performs a flash memory test on the device"
#endif
    "\nflash_memory_update: performs a flash memory update on the device"
    "\nsensor_self_test: performs a sensor self test on the device"
    "\nfactory_calibration_restore: performs a factory calibration "
    "restore on the device";
#ifdef ADIS_BIAS_CORRECTION_UPDATE
  param_desc.description.append(
    "\nbias_correction_update: triggers a bias correction, using the bias "
    "correction factors");
#endif

  m_node->declare_parameter("command_to_execute", "no_command", param_desc);
}

void ImuControlParameters::updateParamsFromHardware()
{
  double doubleDriverParam = 0;
  double doubleCurrentParam = 0;
  int32_t int32DriverParam = 0;
  int32_t int32CurrentParam = 0;

  uint32_t u32DriverParam = 0;
  int64_t int64DriverParam = 0;
  int64_t int64CurrentParam = 0;

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

void ImuControlParameters::handleDoubleParamChange()
{
  double requestedValue = 0;
  double dDriverParam = 0;

  for (const std::string & key : m_attr_current_device) {
    if (m_func_map_get_double_params.find(key) != m_func_map_get_double_params.end()) {
      requestedValue = m_node->get_parameter(key).get_parameter_value().get<double>();

      if (requestedValue != m_double_current_params[key]) {
        // update value in hardware
        const char * ckey = key.c_str();
        if (!(m_iio_wrapper.*(m_func_map_update_double_params[key]))(requestedValue)) {
          RCLCPP_INFO(
            rclcpp::get_logger("rclcpp_imucontrolparameter"),
            "error on set parameter %s with %f value; current value "
            "remained %f",
            ckey, requestedValue, m_double_current_params[key]);
        } else {
          // readback value from hardware
          if (!(m_iio_wrapper.*(m_func_map_get_double_params[key]))(&dDriverParam)) {
            RCLCPP_INFO(
              rclcpp::get_logger("rclcpp_imucontrolparameter"),
              "error on read-back parameter %s, cannot validate if the parameter has been set \
              successfully, please retry",
              ckey);
          } else {
            if (dDriverParam != requestedValue) {
              RCLCPP_INFO(
                rclcpp::get_logger("rclcpp_imucontrolparameter"),
                "Trying to set parameter %s: old value = %f "
                "requested value = %f. "
                "Requested value could not be set in hardware, "
                "instead the following value was set: %f",
                ckey, m_double_current_params[key], requestedValue, dDriverParam);
              m_node->set_parameter(rclcpp::Parameter(ckey, dDriverParam));
            } else {
              RCLCPP_INFO(
                rclcpp::get_logger("rclcpp_imucontrolparameter"),
                "successfully set parameter %s: old "
                "value = %f new value = %f",
                ckey, m_double_current_params[key], requestedValue);
            }
            m_double_current_params[key] = dDriverParam;
          }
        }
      }
    }
  }
}

void ImuControlParameters::handleInt32ParamChange()
{
  int32_t requestedValue = 0;
  int32_t i32DriverParam = 0;

  for (const std::string & key : m_attr_current_device) {
    if (m_func_map_get_int32_params.find(key) != m_func_map_get_int32_params.end()) {
      requestedValue = m_node->get_parameter(key).get_parameter_value().get<int32_t>();

      if (requestedValue != m_int32_current_params[key]) {
        // update value in hardware
        const char * ckey = key.c_str();
        if (!(m_iio_wrapper.*(m_func_map_update_int32_params[key]))(requestedValue)) {
          RCLCPP_INFO(
            rclcpp::get_logger("rclcpp_imucontrolparameter"),
            "error on set parameter %s with %d value; current value "
            "remained %d",
            ckey, requestedValue, m_int32_current_params[key]);
        } else {
          // readback value from hardware
          if (!(m_iio_wrapper.*(m_func_map_get_int32_params[key]))(i32DriverParam)) {
            RCLCPP_INFO(
              rclcpp::get_logger("rclcpp_imucontrolparameter"),
              "error on read-back parameter %s, cannot validate if the parameter has been set \
              successfully, please retry",
              ckey);
          } else {
            if (i32DriverParam != requestedValue) {
              RCLCPP_INFO(
                rclcpp::get_logger("rclcpp_imucontrolparameter"),
                "Trying to set parameter %s: old value = %d "
                "requested value = %d. "
                "Requested value could not be set in hardware, "
                "instead the following value was set: %d",
                ckey, m_int32_current_params[key], requestedValue, i32DriverParam);
              m_node->set_parameter(rclcpp::Parameter(ckey, i32DriverParam));
            } else {
              RCLCPP_INFO(
                rclcpp::get_logger("rclcpp_imucontrolparameter"),
                "successfully set parameter %s: old "
                "value = %d new value = %d",
                ckey, m_int32_current_params[key], requestedValue);
            }
            m_int32_current_params[key] = i32DriverParam;
          }
        }
      }
    }
  }
}

void ImuControlParameters::handleUint32ParamChange()
{
  int64_t requestedValue = 0;
  int64_t i64DriverParam = 0;
  uint32_t u32DriverParam = 0;

  for (const std::string & key : m_attr_current_device) {
    if (m_func_map_get_uint32_params.find(key) != m_func_map_get_uint32_params.end()) {
      requestedValue = m_node->get_parameter(key).get_parameter_value().get<int64_t>();

      if (requestedValue != m_uint32_current_params[key]) {
        // update value in hardware
        const char * ckey = key.c_str();
        if (!(m_iio_wrapper.*(m_func_map_update_uint32_params[key]))(requestedValue)) {
          RCLCPP_INFO(
            rclcpp::get_logger("rclcpp_imucontrolparameter"),
            "error on set parameter %s with %" PRId64 " value; current value remained %" PRId64,
            ckey, requestedValue, m_uint32_current_params[key]);
        } else {
          // readback value from hardware
          if (!(m_iio_wrapper.*(m_func_map_get_uint32_params[key]))(u32DriverParam)) {
            RCLCPP_INFO(
              rclcpp::get_logger("rclcpp_imucontrolparameter"),
              "error on read-back parameter %s, cannot validate if the parameter has been set \
              successfully, please retry",
              ckey);
          } else {
            i64DriverParam = u32DriverParam;
            if (i64DriverParam != requestedValue) {
              RCLCPP_INFO(
                rclcpp::get_logger("rclcpp_imucontrolparameter"),
                "Trying to set parameter %s: old value = "
                "%" PRId64 " requested value = %" PRId64
                ". "
                "Requested value could not be set in hardware, "
                "instead the following value was set: %" PRId64,
                ckey, m_uint32_current_params[key], requestedValue, i64DriverParam);
              m_node->set_parameter(rclcpp::Parameter(ckey, i64DriverParam));
            } else {
              RCLCPP_INFO(
                rclcpp::get_logger("rclcpp_imucontrolparameter"),
                "successfully set parameter %s: old "
                "value = %" PRId64 " new value = %" PRId64,
                ckey, m_uint32_current_params[key], requestedValue);
            }
            m_uint32_current_params[key] = i64DriverParam;
          }
        }
      }
    }
  }
}

void ImuControlParameters::handleCommands()
{
  std::string requestedCommand =
    m_node->get_parameter("command_to_execute").get_parameter_value().get<std::string>();

  if (m_func_map_execute_commands.find(requestedCommand) != m_func_map_execute_commands.end()) {
    m_iio_wrapper.stopBufferAcquisition();
    if (!(m_iio_wrapper.*(m_func_map_execute_commands[requestedCommand]))()) {
      RCLCPP_INFO(
        rclcpp::get_logger("rclcpp_imucontrolparameter"), "error on executing command %s",
        requestedCommand.c_str());
    } else {
      RCLCPP_INFO(
        rclcpp::get_logger("rclcpp_imucontrolparameter"), "executed command %s",
        requestedCommand.c_str());
      // update the rest of parameters based on new changes in hardware
      updateParamsFromHardware();
      m_node->set_parameter(rclcpp::Parameter("command_to_execute", "no_command"));
    }
  } else if (requestedCommand != "no_command") {
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp_imucontrolparameter"), "could not find command %s",
      requestedCommand.c_str());
    m_node->set_parameter(rclcpp::Parameter("command_to_execute", "no_command"));
  }
}

void ImuControlParameters::handleControlParams()
{
  handleCommands();
  handleInt32ParamChange();
  handleUint32ParamChange();
  handleDoubleParamChange();
}
