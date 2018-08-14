#include <engine/AlicaEngine.h>
#include <engine/PlanRepository.h>
#include <engine/collections/RobotProperties.h>
#include <engine/model/CapValue.h>

namespace alica
{

RobotProperties::RobotProperties() {}

RobotProperties::RobotProperties(const AlicaEngine* engine, const std::string& name)
{
    readFromConfig(engine, name);
}

/**
 * Reads the default role as well as the capabilities and characteristics from the Globals.conf.
 */
void RobotProperties::readFromConfig(const AlicaEngine* engine, const std::string& name)
{
    supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

    std::shared_ptr<std::vector<std::string>> caps = (*sc)["Globals"]->getNames("Globals", "Team", name.c_str(), NULL);
    for (const std::string& s : *caps) {
        if (s.compare("ID") == 0 || s.compare("DefaultRole") == 0) {
            continue;
        }
        std::string key = s;
        std::string kvalue = (*sc)["Globals"]->get<std::string>("Globals", "Team", name.c_str(), s.c_str(), NULL);
        for (const Capability* cap : engine->getPlanRepository()->getCapabilities()) {
            if (cap->getName().compare(key) == 0) {
                for (const CapValue* val : cap->getCapValues()) {
                    // transform(kvalue.begin(), kvalue.end(), kvalue.begin(), ::tolower);
                    if (val->getName().compare(kvalue) == 0) {
                        _characteristics.emplace(key, std::unique_ptr<const Characteristic>(new Characteristic(cap, val)));
                    }
                }
            }
        }
    }
    _defaultRole = (*sc)["Globals"]->tryGet<std::string>("NOROLESPECIFIED", "Globals", "Team", name.c_str(), "DefaultRole", NULL);
}

RobotProperties::~RobotProperties() {}

} /* namespace alica */
