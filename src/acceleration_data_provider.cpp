#include "imu_ros2/acceleration_data_provider.h"
#include <sensor_msgs/msg/imu.hpp>

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

sensor_msgs::msg::Imu AccelerationDataProvider::getData(int count)
{
    sensor_msgs::msg::Imu message;
    message.linear_acceleration.x = count;
    message.linear_acceleration.y = 2 * count;
    message.linear_acceleration.z = 3 * count;

    return message;
}
