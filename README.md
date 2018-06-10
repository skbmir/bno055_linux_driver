# bno055_opi_driver

Bosch BNO055 9dof imu ROS node for Orange Pi PC 

should work on compatible boards

------

Parameters:
- bno055_i2c_name - i2c bus file name, default: /dev/i2c-1
- bno055_address - address of sensor on i2c bus, default: 0x29
- bno055_rate - data output rate, default: 50
- ~frame_id - frame id, default: world

Published topics:
- imu/data (sensor_msgs/Imu) - quaternion, linear acceleration and gyro info from bno055 in 9dof mode
- imu/mag (sensor_msgs/MagneticField) - magnetometer info from bno055 in 9dof mode
- imu/temp (sensor_msgs/Temperature) - temperature from bno055 temp sensor

------

# Installation

  Make sure you sources your_catkin_ws/devel/setup.bash

    $ cd your_catkin_ws
    $ source devel/setup.bash

  In your catkin workspace go to src dir and download package sources

    $ cd src
    $ git clone https://github.com/skbmir/bno055_opi_driver

  Then go back to your catkin workspace and build package

    $ cd ..
    $ catkin_make bno055_opi_driver

  After this steps you should be able to run bno055 node

    $ rosrun bno055_opi_driver bno055_node