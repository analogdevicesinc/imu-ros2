/***************************************************************************//**
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
#include <thread>
#include <string>
#include <map>


ImuControlParameters::ImuControlParameters(std::shared_ptr<rclcpp::Node>& node)
{
    declareFunctions();
    init(node);
}

ImuControlParameters::~ImuControlParameters()
{

}

void ImuControlParameters::declareFunctions()
{
    m_funcMap["filter_size"] = &IIOWrapper::update_filter_size;
    m_funcMap["burst_size_selection"] = &IIOWrapper::update_burst_size_selection;
    m_funcMap["burst_data_selection"] = &IIOWrapper::update_burst_data_selection;
    m_funcMap["linear_acceleration_compensation"] = &IIOWrapper::update_linear_acceleration_compensation;
    m_funcMap["point_of_percussion_alignment"] = &IIOWrapper::update_point_of_percussion_alignment;
    m_funcMap["internal_sensor_bandwidth"] = &IIOWrapper::update_internal_sensor_bandwidth;
    m_funcMap["sync_mode_select"] = &IIOWrapper::update_sync_mode_select;
    m_funcMap["sync_polarity"] = &IIOWrapper::update_sync_polarity;
    m_funcMap["data_ready_polarity"] = &IIOWrapper::update_data_ready_polarity;
    m_funcMap["sync_signal_scale"] = &IIOWrapper::update_sync_signal_scale;
    m_funcMap["decimation_filter"] = &IIOWrapper::update_decimation_filter;
    m_funcMap["accel_calibbias_x"] = &IIOWrapper::update_accel_calibbias_x;
    m_funcMap["accel_calibbias_y"] = &IIOWrapper::update_accel_calibbias_y;
    m_funcMap["accel_calibbias_z"] = &IIOWrapper::update_accel_calibbias_z;
    m_funcMap["anglvel_calibbias_x"] = &IIOWrapper::update_anglvel_calibbias_x;
    m_funcMap["anglvel_calibbias_y"] = &IIOWrapper::update_anglvel_calibbias_y;
    m_funcMap["anglvel_calibbias_z"] = &IIOWrapper::update_anglvel_calibbias_z;

    m_funcMap["z_axis_accelerometer_bias_correction_enable"] = &IIOWrapper::update_z_axis_accelerometer_bias_correction_enable;
    m_funcMap["y_axis_accelerometer_bias_correction_enable"] = &IIOWrapper::update_y_axis_accelerometer_bias_correction_enable;
    m_funcMap["x_axis_accelerometer_bias_correction_enable"] = &IIOWrapper::update_x_axis_accelerometer_bias_correction_enable;
    m_funcMap["z_axis_gyroscope_bias_correction_enable"] = &IIOWrapper::update_z_axis_gyroscope_bias_correction_enable;
    m_funcMap["y_axis_gyroscope_bias_correction_enable"] = &IIOWrapper::update_y_axis_gyroscope_bias_correction_enable;
    m_funcMap["x_axis_gyroscope_bias_correction_enable"] = &IIOWrapper::update_x_axis_gyroscope_bias_correction_enable;
    m_funcMap["internal_sync_enable_4khz"] = &IIOWrapper::update_internal_sync_enable_4khz;
    m_funcMap["timestamp32"] = &IIOWrapper::update_timestamp32;
    m_funcMap["fifo_watermark_interrupt_polarity"] = &IIOWrapper::update_fifo_watermark_interrupt_polarity;
    m_funcMap["fifo_watermark_interrupt_enable"] = &IIOWrapper::update_fifo_watermark_interrupt_enable;
    m_funcMap["fifo_overflow_behavior"] = &IIOWrapper::update_fifo_overflow_behavior;
    m_funcMap["fifo_enable"] = &IIOWrapper::update_fifo_enable;
    m_funcMap["bias_correction_time_base_control"] = &IIOWrapper::update_bias_correction_time_base_control;
    m_funcMap["fifo_watermark_threshold_level"] = &IIOWrapper::update_fifo_watermark_threshold_level;

    // for update functions
    m_funcMapGet["filter_size"] = &IIOWrapper::filter_size;
    m_funcMapGet["burst_size_selection"] = &IIOWrapper::burst_size_selection;
    m_funcMapGet["burst_data_selection"] = &IIOWrapper::burst_data_selection;
    m_funcMapGet["linear_acceleration_compensation"] = &IIOWrapper::linear_acceleration_compensation;
    m_funcMapGet["point_of_percussion_alignment"] = &IIOWrapper::point_of_percussion_alignment;
    m_funcMapGet["internal_sensor_bandwidth"] = &IIOWrapper::internal_sensor_bandwidth;
    m_funcMapGet["sync_mode_select"] = &IIOWrapper::sync_mode_select;
    m_funcMapGet["sync_polarity"] = &IIOWrapper::sync_polarity;
    m_funcMapGet["data_ready_polarity"] = &IIOWrapper::data_ready_polarity;
    m_funcMapGet["sync_signal_scale"] = &IIOWrapper::sync_signal_scale;
    m_funcMapGet["decimation_filter"] = &IIOWrapper::decimation_filter;
    m_funcMapGet["accel_calibbias_x"] = &IIOWrapper::accel_x_calibbias;
    m_funcMapGet["accel_calibbias_y"] = &IIOWrapper::accel_y_calibbias;
    m_funcMapGet["accel_calibbias_z"] = &IIOWrapper::accel_z_calibbias;
    m_funcMapGet["anglvel_calibbias_x"] = &IIOWrapper::anglvel_x_calibbias;
    m_funcMapGet["anglvel_calibbias_y"] = &IIOWrapper::anglvel_y_calibbias;
    m_funcMapGet["anglvel_calibbias_z"] = &IIOWrapper::anglvel_z_calibbias;

    m_funcMapGet["z_axis_accelerometer_bias_correction_enable"] = &IIOWrapper::z_axis_accelerometer_bias_correction_enable;
    m_funcMapGet["y_axis_accelerometer_bias_correction_enable"] = &IIOWrapper::y_axis_accelerometer_bias_correction_enable;
    m_funcMapGet["x_axis_accelerometer_bias_correction_enable"] = &IIOWrapper::x_axis_accelerometer_bias_correction_enable;
    m_funcMapGet["z_axis_gyroscope_bias_correction_enable"] = &IIOWrapper::z_axis_gyroscope_bias_correction_enable;
    m_funcMapGet["y_axis_gyroscope_bias_correction_enable"] = &IIOWrapper::y_axis_gyroscope_bias_correction_enable;
    m_funcMapGet["x_axis_gyroscope_bias_correction_enable"] = &IIOWrapper::x_axis_gyroscope_bias_correction_enable;
    m_funcMapGet["internal_sync_enable_4khz"] = &IIOWrapper::internal_sync_enable_4khz;
    m_funcMapGet["timestamp32"] = &IIOWrapper::timestamp32;
    m_funcMapGet["fifo_watermark_interrupt_polarity"] = &IIOWrapper::fifo_watermark_interrupt_polarity;
    m_funcMapGet["fifo_watermark_interrupt_enable"] = &IIOWrapper::fifo_watermark_interrupt_enable;
    m_funcMapGet["fifo_overflow_behavior"] = &IIOWrapper::fifo_overflow_behavior;
    m_funcMapGet["fifo_enable"] = &IIOWrapper::fifo_enable;
    m_funcMapGet["bias_correction_time_base_control"] = &IIOWrapper::bias_correction_time_base_control;
    m_funcMapGet["fifo_watermark_threshold_level"] = &IIOWrapper::fifo_watermark_threshold_level;

    // declare atributes for adis16505
    m_attr_adis16505.push_back("filter_size");
    m_attr_adis16505.push_back("burst_size_selection");
    m_attr_adis16505.push_back("burst_data_selection");
    m_attr_adis16505.push_back("linear_acceleration_compensation");
    m_attr_adis16505.push_back("point_of_percussion_alignment");
    m_attr_adis16505.push_back("internal_sensor_bandwidth");
    m_attr_adis16505.push_back("sync_mode_select");
    m_attr_adis16505.push_back("sync_polarity");
    m_attr_adis16505.push_back("data_ready_polarity");
    m_attr_adis16505.push_back("sync_signal_scale");
    m_attr_adis16505.push_back("decimation_filter");
    m_attr_adis16505.push_back("accel_calibbias_x");
    m_attr_adis16505.push_back("accel_calibbias_y");
    m_attr_adis16505.push_back("accel_calibbias_z");
    m_attr_adis16505.push_back("anglvel_calibbias_x");
    m_attr_adis16505.push_back("anglvel_calibbias_y");
    m_attr_adis16505.push_back("anglvel_calibbias_z");

    // declare atributes for adis16577_3
    m_attr_adis1657x.push_back("filter_size");
    m_attr_adis1657x.push_back("burst_size_selection");
    m_attr_adis1657x.push_back("burst_data_selection");
    m_attr_adis1657x.push_back("linear_acceleration_compensation");
    m_attr_adis1657x.push_back("point_of_percussion_alignment");
    m_attr_adis1657x.push_back("internal_sensor_bandwidth");
    m_attr_adis1657x.push_back("sync_mode_select");
    m_attr_adis1657x.push_back("sync_polarity");
    m_attr_adis1657x.push_back("data_ready_polarity");
    m_attr_adis1657x.push_back("sync_signal_scale");
    m_attr_adis1657x.push_back("decimation_filter");
    m_attr_adis1657x.push_back("accel_calibbias_x");
    m_attr_adis1657x.push_back("accel_calibbias_y");
    m_attr_adis1657x.push_back("accel_calibbias_z");
    m_attr_adis1657x.push_back("anglvel_calibbias_x");
    m_attr_adis1657x.push_back("anglvel_calibbias_y");
    m_attr_adis1657x.push_back("anglvel_calibbias_z");
    m_attr_adis1657x.push_back("z_axis_accelerometer_bias_correction_enable");
    m_attr_adis1657x.push_back("y_axis_accelerometer_bias_correction_enable");
    m_attr_adis1657x.push_back("x_axis_accelerometer_bias_correction_enable");
    m_attr_adis1657x.push_back("z_axis_gyroscope_bias_correction_enable");
    m_attr_adis1657x.push_back("y_axis_gyroscope_bias_correction_enable");
    m_attr_adis1657x.push_back("x_axis_gyroscope_bias_correction_enable");
    m_attr_adis1657x.push_back("internal_sync_enable_4khz");
    m_attr_adis1657x.push_back("timestamp32");
    m_attr_adis1657x.push_back("fifo_watermark_interrupt_polarity");
    m_attr_adis1657x.push_back("fifo_watermark_interrupt_enable");
    m_attr_adis1657x.push_back("fifo_overflow_behavior");
    m_attr_adis1657x.push_back("fifo_enable");
    m_attr_adis1657x.push_back("bias_correction_time_base_control");
    m_attr_adis1657x.push_back("fifo_watermark_threshold_level");

    switch (IIOWrapper::s_device_name_enum) {
    case IIODeviceName::ADIS16505:
        m_attr_current_device = m_attr_adis16505;
        break;
    case IIODeviceName::ADIS1657X:
        m_attr_current_device = m_attr_adis1657x;
        break;
    default:
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp_device_error"), "Device is not supported");
        break;
    }
    }
}

void ImuControlParameters::init(std::shared_ptr<rclcpp::Node> &node)
{
    m_node = node;

    for (const std::string &key: m_attr_current_device)
    {
        int deviceParamValue = (m_iioWrapper.*(m_funcMapGet[key]))();
        m_node->declare_parameter(key, deviceParamValue);
    }

    m_funcMapGetD["sampling_frequency"] = &IIOWrapper::sampling_frequency;
    m_funcMapD["sampling_frequency"] = &IIOWrapper::update_sampling_frequency;

    double deviceParamValued = (m_iioWrapper.*(m_funcMapGetD["sampling_frequency"]))();
    m_node->declare_parameter("sampling_frequency", deviceParamValued);
    m_node->declare_parameter("command_to_execute", "no_command");

    m_funcMap2["software_reset"] = &IIOWrapper::software_reset;
    m_funcMap2["flash_memory_test"] = &IIOWrapper::flash_memory_test;
    m_funcMap2["flash_memory_update"] = &IIOWrapper::flash_memory_update;
    m_funcMap2["sensor_self_test"] = &IIOWrapper::sensor_self_test;
    m_funcMap2["factory_calibration_restore"] = &IIOWrapper::factory_calibration_restore;

    switch (IIOWrapper::s_device_name_enum) {
    case IIODeviceName::ADIS1657X:
        m_funcMap2["fifo_flush"] = &IIOWrapper::fifo_flush;
        m_funcMap2["bias_correction_update"] = &IIOWrapper::bias_correction_update;
        break;
    default:
        break;
    }

}

bool ImuControlParameters::setParameters(std::map<std::string, int32_t>& valueMap, std::map<std::string, double>& valueMapD)
{
    bool hasError = false;

    for (const std::string &key: m_attr_current_device)
    {
        int deviceParamValue = (m_iioWrapper.*(m_funcMapGet[key]))();

        if(deviceParamValue != valueMap[key])
        {
            const char * ckey = key.c_str();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp_parameter_before_set"), "%s: '%d'", ckey , deviceParamValue);

            int ret = (m_iioWrapper.*(m_funcMap[key]))(valueMap[key]);

            if(ret == 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp_set_parameter"), "%s: '%d'", ckey , valueMap[key]);
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp_set_parameter_error"), "error on set parameter %s: '%d' ", ckey , valueMap[key]);
                hasError = true;
            }
        }
    }

    // for sampling_frequency
    {
        double deviceParamValue = (m_iioWrapper.*(m_funcMapGetD["sampling_frequency"]))();

        if(deviceParamValue != valueMapD["sampling_frequency"])
        {
            const char * ckey = "sampling_frequency";

            RCLCPP_INFO(rclcpp::get_logger("rclcpp_parameter_before_set"), "%s: '%f'", ckey , deviceParamValue);

            int ret = (m_iioWrapper.*(m_funcMapD[ckey]))(valueMapD[ckey]);

            if(ret == 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp_set_parameter"), "%s: '%f'", ckey , valueMapD["sampling_frequency"]);
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp_set_parameter_error"), "error on set parameter %s: '%f' ", ckey , valueMapD["sampling_frequency"]);
                hasError = true;
            }
        }
    }

    return hasError;
}

bool ImuControlParameters::triggerCommand(std::string& command_to_execute, bool hasError)
{
    std::vector<std::string> keys;
    for (auto it = m_funcMap2.begin(); it != m_funcMap2.end(); it++)
        keys.push_back(it->first);

    for (const std::string &key: keys)
    {
        if(key == command_to_execute)
        {
            const char * ckey = key.c_str();

            int ret = (m_iioWrapper.*(m_funcMap2[key]))();

            if(ret == 0)
            {
                m_node->set_parameter(rclcpp::Parameter("command_to_execute", "no_command"));
                RCLCPP_INFO(rclcpp::get_logger("rclcpp_set_parameter"), "command_to_execute: '%s'", ckey);
                break;
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp_set_parameter_error"), "command_to_execute error on set %s", ckey );
                hasError = true;
            }
        }
    }
    return hasError;
}

void ImuControlParameters::userParamSettingMode(std::map<std::string, int32_t>& valueMap,
                                                std::map<std::string, double>& valueMapD,
                                                int& lastOperationMode,
                                                bool& wasConfMode)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp_configuration_mode"), "Set ros parameters");

    for (const std::string &key: m_attr_current_device)
        valueMap[key] =  m_node->get_parameter(key).get_parameter_value().get<int32_t>();

    for (const std::string &key: m_attr_current_device)
    {
        int deviceParamValue = (m_iioWrapper.*(m_funcMapGet[key]))();
        int userParamValue = valueMap[key];

        const char * ckey = key.c_str();


        RCLCPP_INFO(rclcpp::get_logger("rclcpp_configuration_mode"),
                    "Current user %s value = %d and current device setting value %d", ckey, userParamValue, deviceParamValue);

    }

    // for sampling_frequency
    {
        valueMapD["sampling_frequency"] =  m_node->get_parameter("sampling_frequency").get_parameter_value().get<double>();
        double deviceParamValue = (m_iioWrapper.*(m_funcMapGetD["sampling_frequency"]))();
        double userParamValue = valueMapD["sampling_frequency"];
        const char * ckey = "sampling_frequency";
        RCLCPP_INFO(rclcpp::get_logger("rclcpp_configuration_mode"),
                    "Current user %s value = %f and current device setting value %f", ckey, userParamValue, deviceParamValue);
    }

    // if user set a command then reset the comand
    std::string command_to_execute = m_node->get_parameter("command_to_execute").get_parameter_value().get<std::string>();
    if(command_to_execute != "no_command")
        m_node->set_parameter(rclcpp::Parameter("command_to_execute", "no_command"));

    lastOperationMode = USER_PARAM_SETTING_MODE;
    wasConfMode = true;
}

void ImuControlParameters::deviceParamSettingMode(std::map<std::string, int32_t>& valueMap,
                                                  std::map<std::string, double>& valueMapD,
                                                  int& lastOperationMode,
                                                  bool& wasConfMode,
                                                  bool& hasError)
{
    if(lastOperationMode == DEVICE_CONTINUOUS_SAMPLING_MODE)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp_write_mode"), "You first go to configuration mode");
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp_write_mode"), "Writting values");

    if(wasConfMode == true)
        hasError = setParameters(valueMap, valueMapD);

    if(hasError == false)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp_write_mode"), "Written values with success");

    for (const std::string &key: m_attr_current_device)
    {
        int deviceParamValue = (m_iioWrapper.*(m_funcMapGet[key]))();

        const char * ckey = key.c_str();

        RCLCPP_INFO(rclcpp::get_logger("rclcpp_write_mode"),
                    "Current %s device setting value %d", ckey, deviceParamValue);

    }

    // for sampling_frequency
    {
        double deviceParamValue = (m_iioWrapper.*(m_funcMapGetD["sampling_frequency"]))();
        const char * ckey = "sampling_frequency";
        RCLCPP_INFO(rclcpp::get_logger("rclcpp_write_mode"),
                    "Current %s device setting value %f", ckey, deviceParamValue);
    }

    std::string command_to_execute = m_node->get_parameter("command_to_execute").get_parameter_value().get<std::string>();

    hasError = triggerCommand(command_to_execute, hasError);

    lastOperationMode = DEVICE_PARAM_SETTING_MODE;
    wasConfMode = false;
}

void ImuControlParameters::run()
{
    std::thread::id this_id = std::this_thread::get_id();
    std::cout << "thread parameter " << this_id << " started...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp_parameter"), "startThread: '%d'", this_id);

    rclcpp::WallRate loopRate(0.1);

    bool hasError = false;

    std::map<std::string, int32_t> valueMap;
    std::map<std::string, double> valueMapD;

    int lastOperationMode = 0;

    bool wasConfMode = false;

    while (rclcpp::ok())
    {
        int32_t operation_mode = m_node->get_parameter("operation_mode").get_parameter_value().get<int32_t>();

        switch(operation_mode) {
        case USER_PARAM_SETTING_MODE:
            userParamSettingMode(valueMap, valueMapD, lastOperationMode, wasConfMode);
            break;
        case DEVICE_PARAM_SETTING_MODE:
            deviceParamSettingMode(valueMap, valueMapD, lastOperationMode, wasConfMode, hasError);
            break;
        case DEVICE_CONTINUOUS_SAMPLING_MODE:
            lastOperationMode = DEVICE_CONTINUOUS_SAMPLING_MODE;
            wasConfMode = false;
            break;
        default:
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp_operaion_mode"), "operation_mode = %d, invalid value, set 0 or 1 or 2", operation_mode);
            break;
        }
        }
        loopRate.sleep();
    }
    this_id = std::this_thread::get_id();
    std::cout << "thread parameter " << this_id << " ended...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp_parameter"), "endThread: '%d'", this_id);
}
