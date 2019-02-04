#pragma once

#include "engine/AgentIDConstPtr.h"
#include "engine/Types.h"
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
    RobotProperties();
    RobotProperties(const AlicaEngine* ae, const std::string& name);
    ~RobotProperties();

    const std::map<std::string, std::unique_ptr<const Characteristic>>& getCharacteristics() const { return _characteristics; }
    const std::string& getDefaultRole() const { return _defaultRole; }

    friend std::ostream& operator<<(std::ostream& os, const alica::RobotProperties& obj)
    {
        os << "RobotProperties: Default Role: " << obj.getDefaultRole() << std::endl;
        for (const std::pair<const std::string, std::unique_ptr<const Characteristic>>& p : obj.getCharacteristics()) {
            os << "\t" << p.first << " = " << p.second->getCapValue()->getName() << std::endl;
        }
        return os;
    }

private:
    void readFromConfig(const AlicaEngine* engine, const std::string& name);

    std::map<std::string, std::unique_ptr<const Characteristic>> _characteristics;
    std::string _defaultRole;
};

} /* namespace alica */
