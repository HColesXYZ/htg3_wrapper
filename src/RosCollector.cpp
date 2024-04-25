#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include "RosCollector.h"

ROSCollector::ROSCollector(DCconfig& config) : DataCollector(config) {}

void ROSCollector::initRPI() {
    if (config.rpi.enabled) {
        rpi_comm = std::make_shared<RPIComms>();
        rpi_comm->initClient(config.rpi.ip.c_str(), config.rpi.port);
        ros_rpi_interface = std::make_shared<ROSRPIInterface>(config.rpi.ip, config.rpi.dataPort);
    }
}

void ROSCollector::startCollectors(){
    if(config.rpi.enabled)
    {
        // Start thread to save data
        mThreads.emplace_back(&ROSRPIInterface::BufferData, ros_rpi_interface.get());
        mThreads.emplace_back(&ROSRPIInterface::PublishData, ros_rpi_interface.get());
    }
    if(config.optitrack.enabled)
    {
        std::string optitrackFN = "";
        ros_opti_interface = std::make_shared<RosOptiInterface>();
        optitrackFN = ros_opti_interface->SetupOptitrack(mOutPath,config.optitrack.ip,config.ipAddress_user,config.optitrack.id);
    }
    sleep(config.rpi.secondsBeforeStart); 
}

void ROSCollector::stopCollectors(){
    if (config.optitrack.enabled)
    {
        //end optitracks
        ros_opti_interface->EndTracking();
        ros_opti_interface->TerminateFileWriting();

        std::cout << "Optitrack stopped" << std::endl;
    }

    // Join all threads
    for (auto& thread : mThreads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    std::cout << "Threads Joined" << std::endl;
}
