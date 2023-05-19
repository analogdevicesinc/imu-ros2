/***************************************************************************//**
*   @file   imu_control_parameters.h
*   @brief  Set ros parameter in libiio
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

#ifndef IMU_CONTROL_PARAMETERS_H
#define IMU_CONTROL_PARAMETERS_H

#include "imu_ros2/ros_task.h"
#include "iio_wrapper.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "setting_declarations.h"

class ImuControlParameters : public RosTask {

public:
    ImuControlParameters(std::shared_ptr<rclcpp::Node>& node);
    ~ImuControlParameters();

    void declareFunctions();

     void init(std::shared_ptr<rclcpp::Node>& node);

     bool setParameters(std::map<std::string, int32_t>& valueMap,
                        std::map<std::string, double> &valueMapD);

     bool triggerCommand(std::string& command_to_execute, bool hasError);

     void userParamSettingMode(std::map<std::string, int32_t>& valueMap,
                               std::map<std::string, double>& valueMapD,
                                                     int& lastOperationMode,
                                                     bool& wasConfMode);

    void deviceParamSettingMode(std::map<std::string, int32_t>& valueMap,
                                std::map<std::string, double>& valueMapD,
                                                       int& lastOperationMode,
                                                       bool& wasConfMode,
                                                       bool& hasError);

     void run() override;

private:
    std::shared_ptr<rclcpp::Node> m_node;
    IIOWrapper m_iioWrapper;

    int32_t m_operation_mode;

    std::string m_command_to_execute;

    typedef int(IIOWrapper::*runFunctionDeclaration)(int32_t);
    typedef std::map<std::string, runFunctionDeclaration> FunctionMapType;
    FunctionMapType m_funcMap;

    typedef int(IIOWrapper::*runFunctionDeclarationGet)();
    typedef std::map<std::string, runFunctionDeclarationGet> FunctionMapTypeGet;
    FunctionMapTypeGet m_funcMapGet;

    typedef int(IIOWrapper::*runFunctionDeclaration2)();
    typedef std::map<std::string, runFunctionDeclaration2> FunctionMapType2;
    FunctionMapType2 m_funcMap2;

    typedef int(IIOWrapper::*runFunctionDeclarationDouble)(double);
    typedef std::map<std::string, runFunctionDeclarationDouble> FunctionMapTypeDouble;
    FunctionMapTypeDouble m_funcMapD;

    typedef double(IIOWrapper::*runFunctionDeclarationGetDouble)();
    typedef std::map<std::string, runFunctionDeclarationGetDouble> FunctionMapTypeGetDouble;
    FunctionMapTypeGetDouble m_funcMapGetD;

    std::map<std::string, int32_t> m_memberMap;

    std::list<std::string> m_attr_adis16505;
    std::list<std::string> m_attr_adis1657x;
    std::list<std::string> m_attr_current_device;
};

#endif // IMU_CONTROL_PARAMETERS_H
