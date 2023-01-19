#ifndef STATIC_DATA_PROVIDER_H
#define STATIC_DATA_PROVIDER_H

#include "imu_ros2/static_data_provider_interface.h"

class StaticDataProvider : public StaticDataProviderInterface {


public:
    StaticDataProvider();
    ~StaticDataProvider();

    void init() override;
    imu_ros2::msg::StaticData getData(int count) override;
};

#endif // STATIC_DATA_PROVIDER_STRING_H
