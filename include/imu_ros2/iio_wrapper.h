#ifndef IIO_WRAPPER_H
#define IIO_WRAPPER_H

#include<string>
#include <iio.h>

class IIOWrapper {
public:
    IIOWrapper();
    ~IIOWrapper();

    float getAccelerometerX();
    float getAccelerometerY();
    float getAccelerometerZ();

    float getGyroscopeX();
    float getGyroscopeY();
    float getGyroscopeZ();

private:
    static struct iio_context * m_network_context;
    struct iio_context * m_object_context;
    struct iio_device *m_dev;

    struct iio_channel *m_channel_accel_x;
    struct iio_channel *m_channel_accel_y;
    struct iio_channel *m_channel_accel_z;

    struct iio_channel *m_channel_anglvel_x;
    struct iio_channel *m_channel_anglvel_y;
    struct iio_channel *m_channel_anglvel_z;
};

#endif // IIO_WRAPPER_H
