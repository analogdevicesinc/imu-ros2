#ifndef GYROSCOPE_DATA_PROVIDER_H
#define GYROSCOPE_DATA_PROVIDER_H

#include "imu_ros2/gyroscope_data_provider_interface.h"

class GyroscopeDataProvider : public GyroscopeDataProviderInterface {


public:
    GyroscopeDataProvider();
    ~GyroscopeDataProvider();

    void init() override;
    imu_ros2::msg::GyroscopeData getData(int count) override;
};

#endif // GYROSCOPE_DATA_PROVIDER_STRING_H
