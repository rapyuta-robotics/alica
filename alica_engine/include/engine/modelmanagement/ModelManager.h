#pragma once

#include "engine/Types.h"
#include <yaml-cpp/yaml.h>

#include <functional>
#include <optional>
#include <string>
#include <vector>

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
    ModelManager(ConfigChangeListener& configChangeListener, const std::vector<std::string>& domainConfigFolders, PlanRepository& planRepository);
    Plan* loadPlanTree(const std::string& masterPlanName, std::optional<std::string> placeholderMapping = std::nullopt);
    RoleSet* loadRoleSet(const std::string& roleSetName);

    bool idExists(const int64_t id) const;

private:
    static constexpr const char* LOGNAME = "ModelManager";

    friend Factory;

    ConfigChangeListener& _configChangeListener;
    const std::vector<std::string> _domainConfigFolders;
    std::list<std::string> filesToParse;
    std::list<std::string> filesParsed;

    // destruction order between elements and planRepository seems important here
    std::map<int64_t, std::unique_ptr<AlicaElement>> elements;
    PlanRepository& _planRepository;

    const AlicaElement* getElement(const int64_t id);
    // Recursively look for a file named <fileName> in _domainConfigFolders & return the absolute path to the file. The returned path is empty if the file is
    // not found
    std::string findFile(const std::string& fileName) const;
    AlicaElement* parseFile(const std::string& currentFile, const std::string& type);
    /**
     * Searches for the default role set in _domainConfigFolders (not recursively)
     * @return The first default role set it finds.
     */
    std::string findDefaultRoleSet() const;
    void attachReferences();
    void generateTemplateVariables();
    void computeReachabilities();
};

} // namespace alica
