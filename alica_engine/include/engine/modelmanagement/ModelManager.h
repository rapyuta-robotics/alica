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
class Factory;

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
    friend Factory;

    essentials::SystemConfig* sc;
    std::string domainConfigFolder;
    std::string basePlanPath;
    std::string baseRolePath;
    std::string baseTaskPath;
    std::list<std::string> filesToParse;
    std::list<std::string> filesParsed;

    PlanRepository* planRepository;
    std::map<int64_t, AlicaElement*> elements;

    const AlicaElement* getElement(const int64_t id);
    const std::string getBasePath(const std::string& configKey);
    AlicaElement* parseFile(const std::string& currentFile, const std::string& type);
    std::string findDefaultRoleSet(const std::string& dir);
    void attachReferences();
    void generateTemplateVariables();
    void computeReachabilities();
};


} // namespace alica
