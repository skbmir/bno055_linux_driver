#include <iostream>
#include <fstream>
#include "bno055_driver.h"

int main(int argc, char** argv)
{
    int bno055_addr = 0x29;
    int rate = 100;
    const char bno055_i2c_dev[] = "/dev/i2c-1";

    BNO055_I2C_init(bno055_i2c_dev, bno055_addr);

    struct bno055_t bno055;

    bno055.bus_read = BNO055_I2C_bus_read;
    bno055.bus_write = BNO055_I2C_bus_write;
    bno055.delay_msec = BNO055_delay_msec;
    bno055.dev_addr = bno055_addr;

    bno055_init(&bno055);

    bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

    for (;;)
    {
        u8 accel_status = 0;
        bno055_get_accel_calib_stat(&accel_status);
        u8 gyro_status = 0;
        bno055_get_gyro_calib_stat(&gyro_status);
        u8 mag_status = 0;
        bno055_get_mag_calib_stat(&mag_status);

        std::string cmd;
        std::cout << "What to do next? (check or save)" << std::endl;
        std::cin >> cmd;

        if (cmd == "check")
        {
            std::cout << "Calibration status:" << std::endl <<
                      "A: " << accel_status << 
                      " G: " << gyro_status << 
                      " M: " << mag_status << std::endl;
        }
        else if (cmd == "save")
        {
            bno055_accel_offset_t accel_offset;
            bno055_gyro_offset_t gyro_offset;
            bno055_mag_offset_t mag_offset;

            bno055_read_accel_offset(&accel_offset);
            bno055_read_gyro_offset(&gyro_offset);
            bno055_read_mag_offset(&mag_offset);

            if (argc < 2)
                exit(1);

            std::ofstream out(argv[1]);

            out << accel_offset.x << "," << accel_offset.y << "," << accel_offset.z << "," << accel_offset.r << std::endl;
            out << gyro_offset.x << "," << gyro_offset.y << "," << gyro_offset.z << "," << std::endl;
            out << mag_offset.x << "," << mag_offset.y << "," << gyro_offset.z << "," << mag_offset.r << std::endl;

            out.close();

            break;
        }
    }

    return 0;
}