#ifndef IIO_WRAPPER_H
#define IIO_WRAPPER_H

#include<string>

class IIOWrapper {
public:
    IIOWrapper();
    ~IIOWrapper();

    std::string ssystem(const char *command,const char *filename);

    float getAccelerometerValue(std::string axa);
    float getAccelerometerX();
    float getAccelerometerY();
    float getAccelerometerZ();
};

#endif // IIO_WRAPPER_H
