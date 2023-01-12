#include "imu_ros2/dataproviderstring.h"

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
    message.data = "Hello, world! " + std::to_string(count);

    return message;
}
