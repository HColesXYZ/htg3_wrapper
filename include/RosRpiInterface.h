#pragma once

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include <condition_variable>

#include "RosMessage.h"
#include "RingBuffer.h"
#include "RPIInterface.h"

class ROSRPIInterface : public RPIInterface {
public:
    ROSRPIInterface(const std::string& ip, int port);
    ~ROSRPIInterface();

    void initDefaultBuffers();
    void BufferData();
    void PublishData();

private:
    std::mutex mADISBufferMutex;
    std::condition_variable mADISBufferReady;

    std::shared_ptr<RingBuffer> mADISBuffer;
    std::shared_ptr<RingBuffer> mADISTrigBuffer;
   
    ros::NodeHandle nh; // ROS node handle
    ros::Publisher adis_publisher; // Publisher object
};
