#pragma once

#include "engine/AlicaEngine.h"

#include <yaml-cpp/yaml.h>

#include <string>

namespace essentials{
    class SystemConfig;
}

namespace alica
{

class PlanRepository;
class AlicaElement;
class Plan;
class RoleSet;

/**
 * Parse the plan tree from disk and writes it back. Fills the PlanRepository and holds all existing elements.
 */
class ModelManager
{
public:
    ModelManager(PlanRepository* planRepository);
    Plan* loadPlanTree(const std::string& masterPlanName);
    RoleSet* loadRoleSet(const std::string& roleSetName);

    bool idExists(const int64_t id);

private:
    essentials::SystemConfig* sc;
    std::string domainConfigFolder;
    std::string basePlanPath;
    std::string baseRolePath;
    std::string baseTaskPath;

    PlanRepository* planRepository;
    std::map<int64_t, AlicaElement*> elements;

    const AlicaElement* getElement(const int64_t id);
    const std::string getBasePath(const std::string& configKey);
    AlicaElement* parseFile(const std::string& currentFile, const std::string& type);



};


} // namespace alica
