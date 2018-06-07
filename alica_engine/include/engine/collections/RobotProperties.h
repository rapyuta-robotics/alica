#pragma once

#include "engine/Types.h"
#include "engine/model/Characteristic.h"
#include <supplementary/AgentID.h>

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
    RobotProperties(AgentIDConstPtr agentId, const AlicaEngine* ae, const std::string& name);
    virtual ~RobotProperties();
    void readFromConfig(const AlicaEngine* engine, const std::string& name);
    AgentIDConstPtr getId() const;
    void setId(AgentIDConstPtr agentId);
    const std::map<std::string, const Characteristic*>& getCharacteristics() const;
    const std::string& getDefaultRole() const;
    void setDefaultRole(const std::string& defaultRole);
    friend std::ostream& operator<<(std::ostream& os, const alica::RobotProperties& obj)
    {
        os << "RobotProperties: Id=" << *obj.getId() << " Default Role: " << obj.getDefaultRole() << std::endl;
        for (const std::pair<std::string, const Characteristic*>& p : obj.getCharacteristics()) {
            os << "\t" << p.first << " = " << p.second->getCapValue()->getName() << std::endl;
        }
        return os;
    }

protected:
    AgentIDConstPtr agentId;

    std::string defaultRole;
    std::map<std::string, const Characteristic*> characteristics;
};

} /* namespace alica */
