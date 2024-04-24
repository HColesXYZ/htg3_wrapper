#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <memory>
#include "ros/ros.h"


#include "RosCollector.h"

int main(int argc, char *argv[])
{
    // Setup ROS Node and read config file
    ros::init(argc, argv, "htg3_wrapper");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::NodeHandle nh;
    std::string node_name = ros::this_node::getName();
    std::string config_param, data_param;
    bool ros_out;

    // Get ROS Params
    nh.param<std::string>(node_name + "/config_param", config_param, "file_not_set");
    nh.param<std::string>(node_name + "/data_param", data_param, "file_not_set");
    nh.param<bool>(node_name + "/ros_out", ros_out, 0);
    // Print ROS Params
    ROS_INFO("Wrapper Version 1.0");
    ROS_INFO("Config Param: %s", config_param.c_str());
    ROS_INFO("Data Param: %s", data_param.c_str());
    ROS_INFO("ROS Out: %d", ros_out);
    
    DCconfig config(config_param.c_str());
    std::string outputFolder(data_param.c_str());

    std::unique_ptr<DataCollector> collector;
    if (ros_out) {
        collector = std::make_unique<ROSCollector>(config);
    } else {
        collector = std::make_unique<DataCollector>(config);
    }

    collector->init();
    std::cout << "Testing" << std::endl;
    return collector->start(outputFolder);
}