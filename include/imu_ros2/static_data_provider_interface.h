#ifndef STATIC_DATA_PROVIDER_INTERFACE_H
#define STATIC_DATA_PROVIDER_INTERFACE_H

#include <string>

//firmware revision, firmware date, product id, serial number, flash memory write counter
//struct StaticData {
//    std::string m_firmwareRevision;
//    std::string m_firmwareDate;
//    std::string m_productId;
//    std::string m_serialNumber;
//    int32_t m_flashMemoryWriteCounter;
//};

#include "imu_ros2/msg/static_data.hpp"

class StaticDataProviderInterface {

public:
    StaticDataProviderInterface(){}
    virtual ~StaticDataProviderInterface(){}

    virtual void init() = 0;
    virtual imu_ros2::msg::StaticData getData(int count) = 0;
};

#endif // STATICN_DATA_PROVIDER_INTERFACE_H
