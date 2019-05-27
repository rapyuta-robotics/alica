#pragma once

#include <essentials/Identifier.h>
#include <string>

namespace essentials
{

class RobotMetaData
{
public:
    RobotMetaData(std::string name, const essentials::Identifier* agentID);
    virtual ~RobotMetaData();

    const essentials::Identifier* agentID;
    std::string name;
};
} /* namespace  essentials */
