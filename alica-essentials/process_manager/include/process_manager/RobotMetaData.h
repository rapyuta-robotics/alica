#pragma once

#include <string>

namespace supplementary {
class AgentID;

class RobotMetaData {
public:
    RobotMetaData(std::string name, const AgentID* agentID);
    virtual ~RobotMetaData();

    const AgentID* agentID;
    std::string name;
};
} /* namespace supplementary */
