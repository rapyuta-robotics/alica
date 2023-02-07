#pragma once

#include "engine/Types.h"

#include <algorithm>
#include <map>
#include <memory>
#include <sstream>
#include <string>

namespace alica
{

class PlanRepository;
struct AgentAnnouncement;

class RobotProperties
{
public:
    RobotProperties(const std::string& defaultRole, const AgentAnnouncement& aa);
    ~RobotProperties() = default;

    const std::string& getDefaultRole() const { return _defaultRole; }

    friend std::ostream& operator<<(std::ostream& os, const alica::RobotProperties& obj)
    {
        os << "RobotProperties: Default Role: " << obj.getDefaultRole() << std::endl;
        return os;
    }

private:
    std::string _defaultRole;
};

} /* namespace alica */
