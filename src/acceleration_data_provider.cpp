#include "imu_ros2/acceleration_data_provider.h"

AccelerationDataProvider::AccelerationDataProvider()
{
    init();
}

AccelerationDataProvider::~AccelerationDataProvider()
{

}

void AccelerationDataProvider::init()
{
    // initialize a library
}

std_msgs::msg::UInt32 AccelerationDataProvider::getData(int count)
{
    auto message = std_msgs::msg::UInt32();
    message.data = count;

    return message;
}
