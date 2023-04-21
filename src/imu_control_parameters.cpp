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
    init(node);
}

ImuControlParameters::~ImuControlParameters()
{

}

void ImuControlParameters::init(std::shared_ptr<rclcpp::Node> &node)
{
    m_node = node;

    m_node->declare_parameter("filter_size", m_iioWrapper.filter_size());
    m_node->declare_parameter("burst_size_selection", m_iioWrapper.burst_size_selection());
    m_node->declare_parameter("burst_data_selection", m_iioWrapper.burst_data_selection());
    m_node->declare_parameter("linear_acceleration_compensation", m_iioWrapper.linear_acceleration_compensation());
    m_node->declare_parameter("point_of_percussion_alignment", m_iioWrapper.point_of_percussion_alignment());
    m_node->declare_parameter("internal_sensor_bandwidth", m_iioWrapper.internal_sensor_bandwidth());
    m_node->declare_parameter("sync_mode_select", m_iioWrapper.sync_mode_select());
    m_node->declare_parameter("sync_polarity", m_iioWrapper.sync_polarity());
    m_node->declare_parameter("data_ready_polarity", m_iioWrapper.data_ready_polarity());
    m_node->declare_parameter("sync_signal_scale", m_iioWrapper.sync_signal_scale());
    m_node->declare_parameter("decimation_filter", m_iioWrapper.decimation_filter());
    m_node->declare_parameter("accel_calibbias_x", m_iioWrapper.accel_x_calibbias());
    m_node->declare_parameter("accel_calibbias_y", m_iioWrapper.accel_y_calibbias());
    m_node->declare_parameter("accel_calibbias_z", m_iioWrapper.accel_z_calibbias());
    m_node->declare_parameter("anglvel_calibbias_x", m_iioWrapper.anglvel_x_calibbias());
    m_node->declare_parameter("anglvel_calibbias_y", m_iioWrapper.anglvel_y_calibbias());
    m_node->declare_parameter("anglvel_calibbias_z", m_iioWrapper.anglvel_z_calibbias());
    m_node->declare_parameter("sampling_frequency", m_iioWrapper.sampling_frequency());
    m_node->declare_parameter("command_to_execute", "no_command");

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
    m_funcMap["sampling_frequency"] = &IIOWrapper::update_sampling_frequency;

    m_funcMap2["software_reset"] = &IIOWrapper::software_reset;
    m_funcMap2["flash_memory_test"] = &IIOWrapper::flash_memory_test;
    m_funcMap2["flash_memory_update"] = &IIOWrapper::flash_memory_update;
    m_funcMap2["sensor_self_test"] = &IIOWrapper::sensor_self_test;
    m_funcMap2["factory_calibration_restore"] = &IIOWrapper::factory_calibration_restore;

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
    m_funcMapGet["sampling_frequency"] = &IIOWrapper::sampling_frequency;
}

bool ImuControlParameters::setParameters(std::map<std::string, int32_t>& valueMap)
{
    bool hasError = false;

    std::vector<std::string> keys;
    for (auto it = m_funcMap.begin(); it != m_funcMap.end(); it++)
        keys.push_back(it->first);

    for (const std::string &key: keys)
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
                                                int& lastOperationMode,
                                                bool& wasConfMode)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp_configuration_mode"), "Set ros parameters");

    std::vector<std::string> keys;
    for (auto it = m_funcMap.begin(); it != m_funcMap.end(); it++)
        keys.push_back(it->first);

    for (const std::string &key: keys)
        valueMap[key] =  m_node->get_parameter(key).get_parameter_value().get<int32_t>();

    for (const std::string &key: keys)
    {
        int deviceParamValue = (m_iioWrapper.*(m_funcMapGet[key]))();
        int userParamValue = valueMap[key];

        const char * ckey = key.c_str();


        RCLCPP_INFO(rclcpp::get_logger("rclcpp_configuration_mode"),
                    "Current user %s value = %d and current device setting value %d", ckey, userParamValue, deviceParamValue);

    }

    // if user set a command then reset the comand
    std::string command_to_execute = m_node->get_parameter("command_to_execute").get_parameter_value().get<std::string>();
    if(command_to_execute != "no_command")
        m_node->set_parameter(rclcpp::Parameter("command_to_execute", "no_command"));

    lastOperationMode = USER_PARAM_SETTING_MODE;
    wasConfMode = true;
}

void ImuControlParameters::deviceParamSettingMode(std::map<std::string, int32_t>& valueMap,
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
        hasError = setParameters(valueMap);

    if(hasError == false)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp_write_mode"), "Written values with success");

    std::vector<std::string> keys;
    for (auto it = m_funcMap.begin(); it != m_funcMap.end(); it++)
        keys.push_back(it->first);

    for (const std::string &key: keys)
    {
        int deviceParamValue = (m_iioWrapper.*(m_funcMapGet[key]))();

        const char * ckey = key.c_str();

        RCLCPP_INFO(rclcpp::get_logger("rclcpp_write_mode"),
                    "Current %s device setting value %d", ckey, deviceParamValue);

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

    int lastOperationMode = 0;

    bool wasConfMode = false;

    while (rclcpp::ok())
    {
        int32_t operation_mode = m_node->get_parameter("operation_mode").get_parameter_value().get<int32_t>();

        switch(operation_mode) {
        case USER_PARAM_SETTING_MODE:
            userParamSettingMode(valueMap, lastOperationMode, wasConfMode);
            break;
        case DEVICE_PARAM_SETTING_MODE:
            deviceParamSettingMode(valueMap, lastOperationMode, wasConfMode, hasError);
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
