#pragma once

#include "engine/Types.h"
#include <yaml-cpp/yaml.h>

#include <functional>
#include <string>

namespace essentials
{
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
class ConfigChangeListener;

/**
 * Parse the plan tree from disk and writes it back. Fills the PlanRepository and holds all existing elements.
 */
class ModelManager
{
public:
    ModelManager(ConfigChangeListener& configChangeListener, const std::string& domainConfigFolder, PlanRepository& planRepository);
    Plan* loadPlanTree(const std::string& masterPlanName);
    RoleSet* loadRoleSet(const std::string& roleSetName);

    bool idExists(const int64_t id) const;
    const EntryPoint* generateIdleEntryPoint();
    void reload(const YAML::Node& config);

private:
    static constexpr const char* LOGNAME = "ModelManager";

    friend Factory;

    ConfigChangeListener& _configChangeListener;
    std::string domainConfigFolder;
    std::string basePlanPath;
    std::string baseRolePath;
    std::string baseTaskPath;
    std::list<std::string> filesToParse;
    std::list<std::string> filesParsed;

    // destruction order between elements and planRepository seems important here
    std::map<int64_t, std::unique_ptr<AlicaElement>> elements;
    PlanRepository& _planRepository;

    const AlicaElement* getElement(const int64_t id);
    std::string getBasePath(const std::string& configKey);
    AlicaElement* parseFile(const std::string& currentFile, const std::string& type);
    std::string findDefaultRoleSet(const std::string& dir);
    void attachReferences();
    void generateTemplateVariables();
    void computeReachabilities();
};

} // namespace alica
