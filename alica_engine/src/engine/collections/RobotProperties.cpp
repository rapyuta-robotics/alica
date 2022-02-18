#include <engine/AlicaEngine.h>
#include <engine/PlanRepository.h>
#include <engine/collections/RobotProperties.h>
#include <engine/containers/AgentAnnouncement.h>

namespace alica
{

RobotProperties::RobotProperties(const AlicaEngine* engine, const std::string& defaultRole, const AgentAnnouncement& aa)
        : _defaultRole(defaultRole.empty() ? "NOROLESPECIFIED" : defaultRole)
{
    // TODO: develop proper Role-Capability-Stuff when we need it
    //    for (const AgentAnnouncement::CapabilityPair& cp : aa.capabilities) {
    //        for (const Capability* cap : engine->getPlanRepository().getCapabilities()) {
    //            if (cap->getName() == cp.first) {
    //                for (const CapValue* val : cap->getCapValues()) {
    //                    // transform(kvalue.begin(), kvalue.end(), kvalue.begin(), ::tolower);
    //                    if (val->getName() == cp.second) {
    //                        _characteristics.emplace(cp.first, std::unique_ptr<const Characteristic>(new Characteristic(cap, val)));
    //                    }
    //                }
    //            }
    //        }
    //    }
}

RobotProperties::~RobotProperties() {}

} /* namespace alica */
