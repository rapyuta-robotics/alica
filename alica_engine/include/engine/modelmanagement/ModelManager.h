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
class AlicaEngine;
class ConfigChangeListener;
class IAlicaLogger;

/**
 * Parse the plan tree from disk and writes it back. Fills the PlanRepository and holds all existing elements.
 */
class ModelManager
{
public:
    //[[deprecated("It will be removed in the last PR")]]
    ModelManager(PlanRepository& planRepository, AlicaEngine* ae, const std::string& domainConfigFolder, IAlicaLogger& logger); // TOBE removed
    ModelManager(ConfigChangeListener& configChangeListener, const std::string& domainConfigFolder, PlanRepository& planRepository, IAlicaLogger& logger);
    Plan* loadPlanTree(const std::string& masterPlanName);
    RoleSet* loadRoleSet(const std::string& roleSetName);

    bool idExists(const int64_t id) const;
    const EntryPoint* generateIdleEntryPoint();
    void reload(const YAML::Node& config);

private:
    friend Factory;

    ConfigChangeListener& _configChangeListener;
    std::string domainConfigFolder;
    std::string basePlanPath;
    std::string baseRolePath;
    std::string baseTaskPath;
    std::list<std::string> filesToParse;
    std::list<std::string> filesParsed;

    IAlicaLogger& _logger;
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
