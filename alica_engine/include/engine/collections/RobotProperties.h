#pragma once

#include "engine/Types.h"

#include <essentials/AgentIDConstPtr.h>
#include <SystemConfig.h>

#include <algorithm>
#include <map>
#include <memory>
#include <sstream>
#include <string>

namespace alica
{

class AlicaEngine;
class Characteristic;
class Capability;

class RobotProperties
{
public:
    RobotProperties();
    RobotProperties(const AlicaEngine* ae, const std::string& name);
    ~RobotProperties();

    const std::string& getDefaultRole() const { return _defaultRole; }

    friend std::ostream& operator<<(std::ostream& os, const alica::RobotProperties& obj)
    {
        os << "RobotProperties: Default Role: " << obj.getDefaultRole() << std::endl;
        return os;
    }

private:
    void readFromConfig(const AlicaEngine* engine, const std::string& name);

    std::string _defaultRole;
};

} /* namespace alica */
