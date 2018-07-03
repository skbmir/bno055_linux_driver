#include <string>
#include <ros/ros.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/time.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <tf/transform_broadcaster.h>
#include <boost/bind.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/shared_ptr.hpp>
#include "bno055_driver.h"

int main(int argc, char *argv[])
{
    // set up ROS
    ros::init(argc, argv, "bno055_node");
    ros::NodeHandle node_h;
    ros::Publisher imu_publisher = node_h.advertise<sensor_msgs::Imu>("imu/data", 1);
    ros::Publisher mag_publisher = node_h.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
    ros::Publisher temp_publisher = node_h.advertise<sensor_msgs::Temperature>("imu/temp", 1);

    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;

    transform.setOrigin(tf::Vector3(0.0, 0.2, 1.0));
    transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    // load parameters
    int bno055_addr;
    ros::param::param("bno055_address", bno055_addr, BNO055_I2C_ADDR2);

    std::string bno055_i2c_name;
    ros::param::param<std::string>("bno055_i2c_name", bno055_i2c_name, "/dev/i2c-1");

    int rate;
    ros::param::param("bno055_rate", rate, 50);

    std::string frame_id;
    ros::param::param<std::string>("~frame_id", frame_id, "world");

    // init i2c device
    BNO055_I2C_init(bno055_i2c_name.c_str(), bno055_addr);

    // init driver
    struct bno055_t BNO_dev; // device handle
    BNO_dev.bus_read = BNO055_I2C_bus_read; // set user defined read function
    BNO_dev.bus_write = BNO055_I2C_bus_write; // set user defined write function
    BNO_dev.delay_msec = BNO055_delay_msec; // set user defined delay function
    BNO_dev.dev_addr = bno055_addr; // set bno055 address

    // init bno055
    bno055_init(&BNO_dev); // driver's basic init routine
    bno055_set_power_mode(BNO055_POWER_MODE_NORMAL); // normal power mode
    bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF); // operation mode if 9dof sensor fusion

    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
        bno055_quaternion_double_t quat;
        bno055_linear_accel_double_t lin_acc;
        bno055_gyro_double_t ang_vel;
        bno055_mag_double_t mag_field;
        double temp;

        bno055_convert_double_quaternion_wxyz(&quat);
        bno055_convert_double_linear_accel_xyz_msq(&lin_acc);
        bno055_convert_double_gyro_xyz_rps(&ang_vel);
        bno055_convert_double_mag_xyz_uT(&mag_field);
        bno055_convert_double_temp_celsius(&temp);

        sensor_msgs::Imu imu_msg;
        sensor_msgs::MagneticField mag_msg;
        sensor_msgs::Temperature temp_msg;

        ros::Time cur_time;
        cur_time = ros::Time::now();

        imu_msg.header.stamp = cur_time;
        imu_msg.header.frame_id = frame_id;
        imu_msg.orientation.w = quat.w;
        imu_msg.orientation.x = quat.x;
        imu_msg.orientation.y = quat.y;
        imu_msg.orientation.z = quat.z;
        imu_msg.linear_acceleration.x = lin_acc.x;
        imu_msg.linear_acceleration.y = lin_acc.y;
        imu_msg.linear_acceleration.z = lin_acc.z;
        imu_msg.angular_velocity.x = ang_vel.x;
        imu_msg.angular_velocity.y = ang_vel.y;
        imu_msg.angular_velocity.z = ang_vel.z;

        mag_msg.header.stamp = cur_time;
        mag_msg.header.frame_id = frame_id;
        mag_msg.magnetic_field.x = mag_field.x * 1000000;
        mag_msg.magnetic_field.y = mag_field.y * 1000000;
        mag_msg.magnetic_field.z = mag_field.z * 1000000;

        temp_msg.header.stamp = cur_time;
        temp_msg.header.frame_id = frame_id;
        temp_msg.temperature = temp;
        temp_msg.variance = 0.0;

        imu_publisher.publish(imu_msg);
        mag_publisher.publish(mag_msg);
        temp_publisher.publish(temp_msg);
        broadcaster.sendTransform(transform, cur_time, "base_imu", "base_link");

        loop_rate.sleep();
    }

    close(bno055_file);

    return 0;
}