#pragma once

#include "engine/IRobotID.h"
#include "engine/model/Characteristic.h"

#include <SystemConfig.h>

#include <algorithm>
#include <map>
#include <memory>
#include <sstream>
#include <string>

namespace alica
{

class Characteristic;
class Capability;
class AlicaEngine;

class RobotProperties
{
  public:
    RobotProperties(const IRobotID *agentId);
    RobotProperties(const IRobotID *agentId, AlicaEngine *ae, string name);
    virtual ~RobotProperties();
    void readFromConfig(AlicaEngine *engine, string name);
    const IRobotID *getId() const;
    void setId(const IRobotID *agentId);
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
    const IRobotID *agentId;

    string defaultRole;
    map<string, Characteristic *> characteristics;
    map<long, Capability *> capabilities;
};

} /* namespace alica */
