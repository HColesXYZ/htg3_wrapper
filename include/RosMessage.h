#pragma once

#include <iostream>
#include <vector>
#include <sensor_msgs/Imu.h>

extern double adi_gyro_scale;
extern double adi_acc_scale;

// Define ADIS class inheriting from sensor_msgs::Imu
class RosMessage {

public:
    // Constructor
    //ADIS(const uint8_t* data, const uint8_t* trigger);
    static void adis(sensor_msgs::Imu& message, const uint8_t* data, const uint8_t* trigger);
    static void adis_print(const sensor_msgs::Imu& message);
};
