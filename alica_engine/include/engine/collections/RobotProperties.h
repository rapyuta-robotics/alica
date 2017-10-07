#pragma once

#include "supplementary/IAgentID.h"
#include "engine/model/Characteristic.h"

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
    RobotProperties(const supplementary::IAgentID *agentId);
    RobotProperties(const supplementary::IAgentID *agentId, AlicaEngine *ae, string name);
    virtual ~RobotProperties();
    void readFromConfig(AlicaEngine *engine, string name);
    const supplementary::IAgentID *getId() const;
    void setId(const supplementary::IAgentID *agentId);
    const map<string, Characteristic *> &getCharacteristics() const;
    const string &getDefaultRole() const;
    void setDefaultRole(const string &defaultRole);
    friend std::ostream &operator<<(std::ostream &os, const alica::RobotProperties &obj)
    {
        os << "RobotProperties: Id=" << obj.getId() << " Default Role: " << obj.getDefaultRole() << endl;
        for (pair<string, Characteristic *> p : obj.getCharacteristics())
        {
            os << "\t" << p.first << " = " << p.second->getCapValue()->getName() << endl;
        }
        return os;
    }

  protected:
    const supplementary::IAgentID *agentId;

    string defaultRole;
    map<string, Characteristic *> characteristics;
    map<long, Capability *> capabilities;
};

} /* namespace alica */
