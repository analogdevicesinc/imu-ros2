#ifndef DATAPROVIDERSTRING_H
#define DATAPROVIDERSTRING_H

#include "imu_ros2/dataproviderinterface.h"

class DataProviderString : public DataProviderInterface {


public:
    DataProviderString();
    ~DataProviderString();

    void init() override;
    std_msgs::msg::String getData(int count) override;
};

#endif // DATAPROVIDERSTRING_H
