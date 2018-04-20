#include <engine/collections/RobotProperties.h>
#include "supplementary/AgentIDFactory.h"
#include <engine/AlicaEngine.h>
#include <engine/PlanRepository.h>
#include <engine/model/CapValue.h>

namespace alica {

RobotProperties::RobotProperties(const supplementary::AgentID* agentId, const AlicaEngine* engine, string name)
        : agentId(agentId) {
    this->readFromConfig(engine, name);
}

/**
 * Reads the default role as well as the capabilities and characteristics from the Globals.conf.
 */
void RobotProperties::readFromConfig(const AlicaEngine* engine, string name) {
    supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
    std::string key = "";
    std::string kvalue = "";
    std::shared_ptr<std::vector<std::string>> caps = (*sc)["Globals"]->getNames("Globals", "Team", name.c_str(), NULL);
    for (const std::string& s : *caps) {
        if (s.compare("ID") == 0 || s.compare("DefaultRole") == 0) {
            continue;
        }
        key = s;
        kvalue = (*sc)["Globals"]->get<std::string>("Globals", "Team", name.c_str(), s.c_str(), NULL);
        for (const Capability* cap : engine->getPlanRepository()->getCapabilities()) {
            if (cap->getName().compare(key) == 0) {
                for (const CapValue* val : cap->getCapValues()) {
                    // transform(kvalue.begin(), kvalue.end(), kvalue.begin(), ::tolower);
                    if (val->getName().compare(kvalue) == 0) {
                        Characteristic* cha = new Characteristic();
                        cha->setCapability(cap);
                        cha->setCapValue(val);
                        this->characteristics.insert(std::pair<std::string, const Characteristic*>(key, cha));
                    }
                }
            }
        }
    }
    this->defaultRole =
            (*sc)["Globals"]->tryGet<string>("NOROLESPECIFIED", "Globals", "Team", name.c_str(), "DefaultRole", NULL);
}

RobotProperties::~RobotProperties() {
    for (auto x : this->characteristics) {
        delete x.second;
    }
}

const supplementary::AgentID* RobotProperties::getId() const {
    return agentId;
}

const std::map<std::string, const Characteristic*>& RobotProperties::getCharacteristics() const {
    return this->characteristics;
}

const std::string& RobotProperties::getDefaultRole() const {
    return defaultRole;
}

void RobotProperties::setDefaultRole(const std::string& defaultRole) {
    this->defaultRole = defaultRole;
}

} /* namespace alica */
