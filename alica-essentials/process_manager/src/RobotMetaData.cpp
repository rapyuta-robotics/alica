#include "process_manager/RobotMetaData.h"

namespace essentials
{

RobotMetaData::RobotMetaData(std::string name, const essentials::AgentID* agentID)
        : name(name)
        , agentID(agentID)
{
}

RobotMetaData::~RobotMetaData()
{
    delete agentID;
}

} /* namespace  essentials */
