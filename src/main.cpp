#include <memory>
#include "ros/ros.h"

#include "data_collector.h"

bool getROSParam(const ros::NodeHandle& nh, const std::string& node_name, const std::string& param_name, std::string& param_value) {

    std::string param_name_full = "/" + node_name + "/" + param_name;

    if (!nh.getParam(param_name_full, param_value)) {
        ROS_ERROR("Failed to get parameter '%s'", param_name.c_str());
        return false;
    }
    return true;
}

int main(int argc, char *argv[])
{
    // Setup ROS Node and read config file
    ros::init(argc, argv, "htg3_wrapper");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::NodeHandle nh;
    std::string node_name = ros::this_node::getName();
    std::string config_param, data_param;

    if (!getROSParam(nh, node_name, "data_param", data_param)) return 1;
    if (!getROSParam(nh, node_name, "config_param", config_param)) return 1;
    ROS_INFO("Wrapper Version 1.0");
    ROS_INFO("Config Param: %s", config_param.c_str());
    ROS_INFO("Data Param: %s", data_param.c_str());

    // test to see if library link ok
    DCconfig config(config_param.c_str());
    std::string outputFolder(data_param.c_str());
    return start_collecting(argc, argv, config, outputFolder);
}