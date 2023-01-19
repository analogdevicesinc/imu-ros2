#ifndef DATA_PROVIDER_STRING_H
#define DATA_PROVIDER_STRING_H

#include "imu_ros2/data_provider_interface.h"

class DataProviderString : public DataProviderInterface {


public:
    DataProviderString();
    ~DataProviderString();

    void init() override;
    std_msgs::msg::String getData(int count) override;
};

#endif // DATA_PROVIDER_STRING_H