#ifndef BNO055_DRIVER_H
#define BNO055_DRIVER_H

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <time.h>
#include "bno055.h"

#define MAX_I2C_NAME_LEN 20
#define MAX_RDWR_BUF_LEN 10

// need to be declared in global
// due to bno055 driver library specifics
int bno055_file;

/*!
* struct for Quaternion data in double precision floating point format
*/
struct bno055_quaternion_double_t {
    double w; /** Quaternion w data */
    double x; /** Quaternion x data */
    double y; /** Quaternion y data */
    double z; /** Quaternion z data */
};

// init bno055 device
void BNO055_I2C_init(const char *filename, int dev_address)
{
    bno055_file = open(filename, O_RDWR);
    if (bno055_file < 0)
    {
        printf("Failed to open i2c.\n");
        exit(EXIT_FAILURE);
    }

    if (ioctl(bno055_file, I2C_SLAVE, dev_address) < 0)
    {
        printf("Failed to connect to slave device.\n");
        close(bno055_file);
        exit(EXIT_FAILURE);
    }
}

/*  brief: function used as I2C bus write
 *  return : Status of the I2C write
 *  dev_addr : The device address of the sensor
 *  reg_addr : Address of the first register,
 *    to which data is going to be written
 *  reg_data : It is a value hold in the array,
 *    which will be used for write the value into the register
 *  cnt : The number of bytes of data to be written
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s8 ret = BNO055_SUCCESS;
    u8 buf[MAX_RDWR_BUF_LEN];
    u8 datapos;

    if ((cnt + 1) > MAX_RDWR_BUF_LEN)
    {
        ret = BNO055_ERROR;
        return ret;
    }

    // prepare buffer
    buf[0] = reg_addr; // set register address
    for (datapos = 0; datapos < cnt; datapos++) // VVV
        buf[datapos + 1] = reg_data[datapos];   // copy data to buffer

    // try to write data to bno055
    if (write(bno055_file, buf, cnt + 1) != (cnt + 1))
        ret = BNO055_ERROR;

    return ret;
}

/*  brief: The API is used as I2C bus read
 *  return : Status of the I2C read
 *  dev_addr : The device address of the sensor
 *  reg_addr : Address of the first register,
 *    will data is going to be read
 *  reg_data : This data read from the sensor,
 *    which is hold in an array
 *  cnt : The number of bytes of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s8 ret = BNO055_SUCCESS;
    u8 buf[MAX_RDWR_BUF_LEN];
    u8 datapos;

    if (cnt > MAX_RDWR_BUF_LEN)
    {
        ret = BNO055_ERROR;
        return ret;
    }

    // set register address to write to
    buf[0] = reg_addr;
    if (write(bno055_file, buf, 1) != 1) // if can't set register address
    {
        ret = BNO055_ERROR; // VVV
        return ret;         // return error code
    }

    // try to read data from bno055
    if (read(bno055_file, buf, cnt) != cnt)
    {
        ret = BNO055_ERROR;
        return ret;
    }

    // copy data from buffer
    for (datapos = 0; datapos < cnt; datapos++)
        reg_data[datapos] = buf[datapos];

    return ret;
}

/*  brief : The delay routine
 *  msec : delay in milliseconds
*/
void BNO055_delay_msec(u32 msec)
{
    if ((msec > 999) || (msec < 0))
    {
        printf("Delay milliseconds value is out of allowed range.");
    }

    struct timespec wait_time;
    wait_time.tv_sec = 0;
    wait_time.tv_nsec = msec * 1000000;

    nanosleep(&wait_time, NULL);
}

// read raw quaternion data from bno055 and convert to double format
void bno055_convert_double_quaternion_wxyz(bno055_quaternion_double_t *quat_data)
{
    bno055_quaternion_t raw_quat;
    const double scale = 1.0 / (1 << 14); // see bno055 datasheet -> 3.6.5.5 Orientation (Quaternion)
                                          // Quaternion data representation
                                          // 1 quaternion = 2^14 raw

    bno055_read_quaternion_wxyz(&raw_quat);

    quat_data->w = (double)(raw_quat.w) * scale;
    quat_data->x = (double)(raw_quat.x) * scale;
    quat_data->y = (double)(raw_quat.y) * scale;
    quat_data->z = (double)(raw_quat.z) * scale;
}

#endif // BNO055_DRIVER_H
