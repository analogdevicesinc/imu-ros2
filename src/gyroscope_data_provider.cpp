#include "imu_ros2/gyroscope_data_provider.h"

GyroscopeDataProvider::GyroscopeDataProvider()
{
    init();
}

GyroscopeDataProvider::~GyroscopeDataProvider()
{

}

void GyroscopeDataProvider::init()
{
    // initialize a library
}

imu_ros2::msg::GyroscopeData GyroscopeDataProvider::getData(int count)
{
    (int)count;
    imu_ros2::msg::GyroscopeData message;
    message.anglvel_x =m_iioWrapper.getGyroscopeX();
    message.anglvel_y =m_iioWrapper.getGyroscopeY();
    message.anglvel_z =m_iioWrapper.getGyroscopeZ();


    return message;
}
