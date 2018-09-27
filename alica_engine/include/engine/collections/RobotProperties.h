#pragma once

#include "engine/model/Characteristic.h"
#include "supplementary/AgentID.h"

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
    RobotProperties(const supplementary::AgentID* agentId, const AlicaEngine* ae, const std::string& name);
    virtual ~RobotProperties();
    void readFromConfig(const AlicaEngine* engine, const std::string& name);
    const supplementary::AgentID* getId() const;
    void setId(const supplementary::AgentID* agentId);
    const std::map<std::string, const Characteristic*>& getCharacteristics() const;
    const std::string& getDefaultRole() const;
    void setDefaultRole(const std::string& defaultRole);
    friend std::ostream& operator<<(std::ostream& os, const alica::RobotProperties& obj)
    {
        os << "RobotProperties: Id=" << obj.getId() << " Default Role: " << obj.getDefaultRole() << std::endl;
        for (const std::pair<std::string, const Characteristic*>& p : obj.getCharacteristics()) {
            os << "\t" << p.first << " = " << p.second->getCapValue()->getName() << std::endl;
        }
        return os;
    }

  protected:
    const supplementary::AgentID* agentId;

    std::string defaultRole;
    std::map<std::string, const Characteristic*> characteristics;
};

} /* namespace alica */
