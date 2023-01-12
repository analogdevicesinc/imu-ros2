#ifndef DATAPROVIDERINTERFACE_H
#define DATAPROVIDERINTERFACE_H

#include <std_msgs/msg/string.hpp>

class DataProviderInterface {

public:
    DataProviderInterface(){}
    virtual ~DataProviderInterface(){}

    virtual void init() = 0;
    virtual std_msgs::msg::String getData(int count) = 0;
};

#endif // DATAPROVIDERINTERFACE_H
