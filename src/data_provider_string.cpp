#include "imu_ros2/data_provider_string.h"

DataProviderString::DataProviderString()
{
    init();
}

DataProviderString::~DataProviderString()
{

}

void DataProviderString::init()
{
    // initialize a library
}

std_msgs::msg::String DataProviderString::getData(int count)
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count) + " " + m_parameter;

    return message;
}

void DataProviderString::setParameter(std::string param)
{
    m_parameter = param;
}
