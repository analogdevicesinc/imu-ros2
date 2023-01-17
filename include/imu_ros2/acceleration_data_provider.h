#ifndef ACCELERATION_DATA_PROVIDER_H
#define ACCELERATION_DATA_PROVIDER_H

#include "imu_ros2/acceleration_data_provider_interface.h"

class AccelerationDataProvider : public AccelerationDataProviderInterface {


public:
    AccelerationDataProvider();
    ~AccelerationDataProvider();

    void init() override;
    sensor_msgs::msg::Imu getData(int count) override;
};

#endif // ACCELERATION_DATA_PROVIDER_STRING_H
