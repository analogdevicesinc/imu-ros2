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
    message.linear_acceleration.x = m_iioWrapper.getAccelerometerX();
    message.linear_acceleration.y = m_iioWrapper.getAccelerometerY();
    message.linear_acceleration.z = m_iioWrapper.getAccelerometerZ();

    return message;
}
