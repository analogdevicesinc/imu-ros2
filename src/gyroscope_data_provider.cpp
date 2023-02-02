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
    message.gyroscope_value = 10;


    return message;
}
