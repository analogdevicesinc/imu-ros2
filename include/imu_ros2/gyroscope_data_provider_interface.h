#ifndef GYROSCOPE_DATA_PROVIDER_INTERFACE_H
#define GYROSCOPE_DATA_PROVIDER_INTERFACE_H

#include <string>
#include "imu_ros2/msg/gyroscope_data.hpp"

class GyroscopeDataProviderInterface {

public:
    GyroscopeDataProviderInterface(){}
    virtual ~GyroscopeDataProviderInterface(){}

    virtual void init() = 0;
    virtual imu_ros2::msg::GyroscopeData getData(int count) = 0;
};

#endif // GYROSCOPE_DATA_PROVIDER_INTERFACE_H
