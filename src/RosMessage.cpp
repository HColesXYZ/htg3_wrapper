#include "RosMessage.h"
#include <cmath>

const double pi = 3.14159265358979323846;
const double gravity = 9.81; // m/s^2

double adi_gyro_scale = (pi / 180.0) / (40.0 * pow(2, 16));
double adi_acc_scale = (gravity * 0.00025) / pow(2, 16);

void RosMessage::adis(sensor_msgs::Imu& message, const uint8_t* data, const uint8_t* trigger) {
    // Extract gyro data
    message.angular_velocity.x = static_cast<float>((static_cast<int>(data[26]) << 24) | (static_cast<int>(data[27]) << 16) | (static_cast<int>(data[28]) << 8) | static_cast<int>(data[29])) * adi_gyro_scale;
    message.angular_velocity.y = static_cast<float>((static_cast<int>(data[30]) << 24) | (static_cast<int>(data[31]) << 16) | (static_cast<int>(data[32]) << 8) | static_cast<int>(data[33])) * adi_gyro_scale;
    message.angular_velocity.z = static_cast<float>((static_cast<int>(data[34]) << 24) | (static_cast<int>(data[35]) << 16) | (static_cast<int>(data[36]) << 8) | static_cast<int>(data[37])) * adi_gyro_scale;

    // Extract accelerometer data
    message.linear_acceleration.x = static_cast<float>((static_cast<int>(data[38]) << 24) | (static_cast<int>(data[39]) << 16) | (static_cast<int>(data[40]) << 8) | static_cast<int>(data[41])) * adi_acc_scale;
    message.linear_acceleration.y = static_cast<float>((static_cast<int>(data[42]) << 24) | (static_cast<int>(data[43]) << 16) | (static_cast<int>(data[44]) << 8) | static_cast<int>(data[45])) * adi_acc_scale;
    message.linear_acceleration.z = static_cast<float>((static_cast<int>(data[46]) << 24) | (static_cast<int>(data[47]) << 16) | (static_cast<int>(data[48]) << 8) | static_cast<int>(data[49])) * adi_acc_scale;

    // Set the frame ID
    message.header.frame_id = "ADIS";

    char stampStr[17];
    std::memcpy(stampStr, trigger, 16);
    stampStr[16] = '\0';
    double time = std::strtod(stampStr, nullptr) / 1e6 - 0.0031;
    message.header.stamp = ros::Time(time);
}

// Print function
void RosMessage::adis_print(const sensor_msgs::Imu& message) {
    std::cout << "ADIS Message:" << std::endl;
    std::cout << "  Timestamp: " << message.header.stamp << std::endl;
    std::cout << "  Frame ID: " << message.header.frame_id << std::endl;
    std::cout << "  Angular Velocity (x, y, z): " << message.angular_velocity.x << ", " << message.angular_velocity.y << ", " << message.angular_velocity.z << std::endl;
    std::cout << "  Linear Acceleration (x, y, z): " << message.linear_acceleration.x << ", " << message.linear_acceleration.y << ", " << message.linear_acceleration.z << std::endl;
}