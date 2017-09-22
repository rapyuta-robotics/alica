#include "engine/IRobotIDFactory.h"
#include <engine/AlicaEngine.h>
#include <engine/PlanRepository.h>
#include <engine/collections/RobotProperties.h>
#include <engine/model/CapValue.h>

namespace alica
{

RobotProperties::RobotProperties(const IRobotID *agentId)
    : agentId(agentId)
{
}

RobotProperties::RobotProperties(const IRobotID *agentId, AlicaEngine *engine, string name)
    : agentId(agentId)
{
    this->readFromConfig(engine, name);
}

/**
 * Reads the default role as well as the capabilities and characteristics from the Globals.conf.
 */
void RobotProperties::readFromConfig(AlicaEngine *engine, string name)
{
    supplementary::SystemConfig *sc = supplementary::SystemConfig::getInstance();
    this->characteristics = map<string, Characteristic *>();
    this->capabilities = engine->getPlanRepository()->getCapabilities();
    string key = "";
    string kvalue = "";
    shared_ptr<vector<string>> caps = (*sc)["Globals"]->getNames("Globals", "Team", name.c_str(), NULL);
    for (string s : *caps)
    {
        if (s.compare("ID") == 0 || s.compare("DefaultRole") == 0)
        {
            continue;
        }
        key = s;
        kvalue = (*sc)["Globals"]->get<string>("Globals", "Team", name.c_str(), s.c_str(), NULL);
        for (auto p : this->capabilities)
        {
            if (p.second->getName().compare(key) == 0)
            {
                for (CapValue *val : p.second->getCapValues())
                {
                    // transform(kvalue.begin(), kvalue.end(), kvalue.begin(), ::tolower);
                    if (val->getName().compare(kvalue) == 0)
                    {
                        Characteristic *cha = new Characteristic();
                        cha->setCapability(p.second);
                        cha->setCapValue(val);
                        this->characteristics.insert(pair<string, Characteristic *>(key, cha));
                    }
                }
            }
        }
    }
    this->defaultRole =
        (*sc)["Globals"]->tryGet<string>("NOROLESPECIFIED", "Globals", "Team", name.c_str(), "DefaultRole", NULL);
}

RobotProperties::~RobotProperties()
{
    for (auto x : this->characteristics)
    {
        delete x.second;
    }
}

const IRobotID *RobotProperties::getId() const
{
    return agentId;
}

const map<string, Characteristic *> &RobotProperties::getCharacteristics() const
{
    return this->characteristics;
}

const string &RobotProperties::getDefaultRole() const
{
    return defaultRole;
}

void RobotProperties::setDefaultRole(const string &defaultRole)
{
    this->defaultRole = defaultRole;
}

} /* namespace alica */
