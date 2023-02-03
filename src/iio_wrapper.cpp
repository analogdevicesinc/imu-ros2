#include "imu_ros2/iio_wrapper.h"

#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <sstream>

IIOWrapper::IIOWrapper()
{

}

IIOWrapper::~IIOWrapper()
{

}

std::string IIOWrapper::ssystem(const char *command,const char *filename)
{

    std::string scommand = command;
    std::string cmd = scommand + " > " + filename;
    std::system(cmd.c_str());
    std::ifstream file(filename, std::ios::out | std::ios::binary );
    std::string result;
    if (file) {
        while (!file.eof())
            result.push_back(file.get());
        file.close();
    }
    return result;
}

float IIOWrapper::getAccelerometerValue(std::string axa)
{
    std::string command = "iio_attr -u ip:127.0.0.1 -c adis16505  " + axa + "  raw ";
    std::string readerfile = "tmpfileoutput"+axa+".txt";
    std::string output = ssystem(command.c_str(), readerfile.c_str());
    float num_float = std::stof(output);
    return num_float;
}

float IIOWrapper::getAccelerometerX()
{
    return getAccelerometerValue("accel_x");
}

float IIOWrapper::getAccelerometerY()
{
    return getAccelerometerValue("accel_y");
}

float IIOWrapper::getAccelerometerZ()
{
    return getAccelerometerValue("accel_z");
}

float IIOWrapper::getGyroscopeValue(std::string axa)
{
    std::string command = "iio_attr -u ip:127.0.0.1 -c adis16505  " + axa + "  raw ";
    std::string readerfile = "tmpfileoutput"+axa+".txt";
    std::string output = ssystem(command.c_str(), readerfile.c_str());
    float num_float = std::stof(output);
    return num_float;
}

float IIOWrapper::getGyroscopeX()
{
    return getGyroscopeValue("anglvel_x");
}

float IIOWrapper::getGyroscopeY()
{
    return getGyroscopeValue("anglvel_y");
}

float IIOWrapper::getGyroscopeZ()
{
    return getGyroscopeValue("anglvel_z");
}
