#pragma once

#include <memory>

#include "data_collector.h"
#include "RosRpiInterface.h"
#include "RosOptiInterface.h"

class ROSCollector : public DataCollector {
public:
    ROSCollector(DCconfig& config);

protected:
    void initRPI() override;
    void startCollectors() override;
    void stopCollectors() override;

    std::shared_ptr<ROSRPIInterface> ros_rpi_interface;
    std::shared_ptr<RosOptiInterface> ros_opti_interface;
};
