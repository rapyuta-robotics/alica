#pragma once

#include <string>

#include <yaml-cpp/yaml.h>

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

    bool idExists(int64_t id);

private:
    essentials::SystemConfig* sc;
    std::string domainConfigFolder;
    std::string basePlanPath;
    std::string baseRolePath;
    std::string baseTaskPath;

    PlanRepository* planRepository;
    std::map<int64_t, AlicaElement*> elements;

    using ReferenceList = std::list<std::pair<int64_t, int64_t>>;

    ReferenceList stateInTransitionReferences;
    ReferenceList stateOutTransitionReferences;
    ReferenceList statePlanReferences;
    ReferenceList transitionSynchReferences;
    ReferenceList transitionAimReferences;
    ReferenceList paramSubPlanReferences;
    ReferenceList paramSubVarReferences;
    ReferenceList paramVarReferences;
    ReferenceList conditionVarReferences;
    ReferenceList quantifierScopeReferences;
    ReferenceList epStateReferences;
    ReferenceList epTaskReferences;
    ReferenceList planTypePlanReferences;
    ReferenceList rtmRoleReferences;
    ReferenceList charCapReferences;
    ReferenceList charCapValReferences;
    ReferenceList planningProblemPlanReferences;
    ReferenceList planningProblemPlanWaitReferences;
    ReferenceList planningProblemPlanAlternativeReferences;

    const std::string getBasePath(const std::string& configKey);
    AlicaElement* parseFile(const std::string& currentFile, const std::string& type);

    Plan* createPlan(const YAML::Node& node);
    void createEntryPoints(const YAML::Node& entryPoints, Plan* plan);

    int64_t getReferencedId(const YAML::Node& node);
    AlicaElement* getElement(const int64_t id);
    int64_t getId(const YAML::Node& node);
    std::string getName(const YAML::Node& node);
    std::string getComment(const YAML::Node& node);

    void storeElement(AlicaElement* ael, const std::string& type);

};
} // namespace alica
