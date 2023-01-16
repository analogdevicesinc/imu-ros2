#ifndef DATA_PROVIDER_INTERFACE_H
#define DATA_PROVIDER_INTERFACE_H

#include <std_msgs/msg/string.hpp>

class DataProviderInterface {

public:
    DataProviderInterface(){}
    virtual ~DataProviderInterface(){}

    virtual void init() = 0;
    virtual std_msgs::msg::String getData(int count) = 0;
};

#endif // DATA_PROVIDER_INTERFACE_H
