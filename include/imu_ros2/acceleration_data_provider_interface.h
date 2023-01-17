#ifndef ACCELERATION_DATA_PROVIDER_INTERFACE_H
#define ACCELERATION_DATA_PROVIDER_INTERFACE_H

#include <sensor_msgs/msg/imu.hpp>

class AccelerationDataProviderInterface {

public:
    AccelerationDataProviderInterface(){}
    virtual ~AccelerationDataProviderInterface(){}

    virtual void init() = 0;
    virtual sensor_msgs::msg::Imu getData(int count) = 0;
};

#endif // ACCELERATION_DATA_PROVIDER_INTERFACE_H
