#include "process_manager/RobotMetaData.h"

#include <supplementary/AgentID.h>

namespace supplementary {

RobotMetaData::RobotMetaData(std::string name, const AgentID* agentID)
        : name(name)
        , agentID(agentID) {}

RobotMetaData::~RobotMetaData() {
    delete agentID;
}

} /* namespace supplementary */
