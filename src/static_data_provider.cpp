#include "imu_ros2/static_data_provider.h"

StaticDataProvider::StaticDataProvider()
{
    init();
}

StaticDataProvider::~StaticDataProvider()
{

}

void StaticDataProvider::init()
{
    // initialize a library
}

imu_ros2::msg::StaticData StaticDataProvider::getData(int count)
{
    (int)count;
    imu_ros2::msg::StaticData message;
    message.firmware_revision = "FIRMWARE_REVISION_1";
    message.firmware_date = "19-01-2023";
    message.product_id = "PRODUCT_ID_1";
    message.serial_number = "SERIAL_NUMBER_1";
    message.flash_memory_write_counter = 1;

    return message;
}
