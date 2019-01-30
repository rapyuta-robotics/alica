#pragma once

#include <essentials/AgentID.h>
#include <string>

namespace essentials
{

class RobotMetaData
{
public:
    RobotMetaData(std::string name, const essentials::AgentID* agentID);
    virtual ~RobotMetaData();

    const essentials::AgentID* agentID;
    std::string name;
};
} /* namespace  essentials */
