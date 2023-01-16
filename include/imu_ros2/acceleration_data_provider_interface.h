#ifndef ACCELERATION_DATA_PROVIDER_INTERFACE_H
#define ACCELERATION_DATA_PROVIDER_INTERFACE_H

#include <std_msgs/msg/u_int32.hpp>

class AccelerationDataProviderInterface {

public:
    AccelerationDataProviderInterface(){}
    virtual ~AccelerationDataProviderInterface(){}

    virtual void init() = 0;
    virtual std_msgs::msg::UInt32 getData(int count) = 0;
};

#endif // ACCELERATION_DATA_PROVIDER_INTERFACE_H
