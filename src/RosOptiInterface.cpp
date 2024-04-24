#include "RosOptiInterface.h"

// Definition of the static member
std::shared_ptr<geometry_msgs::PoseStamped> RosOptiInterface::static_pose;
ros::Publisher RosOptiInterface::opti_publisher;

RosOptiInterface::RosOptiInterface() : OptitrackInterface() {
    // Initialize the static pose
    static_pose = std::make_shared<geometry_msgs::PoseStamped>();
    static_pose->header.frame_id = "Opti"; // Set your frame id here

    opti_publisher = nh.advertise<geometry_msgs::PoseStamped>("/opti", 10);
}

RosOptiInterface::~RosOptiInterface()
{
    opti_publisher.shutdown();
}

void RosOptiInterface::setCallbackFunction()
{
    g_pClient->SetFrameReceivedCallback(&RosOptiInterface::DataPublisher, g_pClient); // this function will receive data from the server
}
