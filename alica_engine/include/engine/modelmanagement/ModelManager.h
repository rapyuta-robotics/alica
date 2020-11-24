#pragma once

#include "engine/ConfigChangeListener.h"

#include <yaml-cpp/yaml.h>

#include <string>

namespace essentials{
    class SystemConfig;
}

namespace alica
{

class PlanRepository;
class EntryPoint;
class AlicaElement;
class Plan;
class RoleSet;
class Factory;
class AlicaEngine;

/**
 * Parse the plan tree from disk and writes it back. Fills the PlanRepository and holds all existing elements.
 */
class ModelManager : ConfigChangeListener
{
public:
    ModelManager(PlanRepository& planRepository, AlicaEngine* ae);
    Plan* loadPlanTree(const std::string& masterPlanName);
    RoleSet* loadRoleSet(const std::string& roleSetName);

    bool idExists(const int64_t id) const;
    const EntryPoint* generateIdleEntryPoint();
    void reload(const YAML::Node& config) override;

private:
    friend Factory;

    AlicaEngine* _ae;
    std::string domainConfigFolder;
    std::string basePlanPath;
    std::string baseRolePath;
    std::string baseTaskPath;
    std::list<std::string> filesToParse;
    std::list<std::string> filesParsed;

    PlanRepository& _planRepository;
    std::map<int64_t, AlicaElement*> elements;

    const AlicaElement* getElement(const int64_t id);
    std::string getBasePath(const std::string& configKey);
    AlicaElement* parseFile(const std::string& currentFile, const std::string& type);
    std::string findDefaultRoleSet(const std::string& dir);
    void attachReferences();
    void generateTemplateVariables();
    void computeReachabilities();
};


} // namespace alica
