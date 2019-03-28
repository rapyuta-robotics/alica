#pragma once

#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"

#include <yaml-cpp/yaml.h>

#include <map>

namespace alica
{
using ReferenceList = std::list<std::pair<int64_t, int64_t>>;
class AlicaElement;
class Factory
{
public:
    template <typename T>
    static T getValue(const YAML::Node& node, const std::string& key);
    template <typename T>
    static T getValue(const YAML::Node& node, const std::string& key, T defaultValue);
    static int64_t getReferencedId(const YAML::Node& node);
    static void setElementsMap(std::map<int64_t, AlicaElement*>& elements);
    static void setPlanRepository(PlanRepository* planRepository);
    static void storeElement(AlicaElement* ael, const std::string& type);
    static void setAttributes(const YAML::Node& node, AlicaElement* ael);

protected:
    static ReferenceList stateInTransitionReferences;
    static ReferenceList stateOutTransitionReferences;
    static ReferenceList statePlanReferences;
    static ReferenceList transitionSynchReferences;
    static ReferenceList transitionInStateReferences;
    static ReferenceList transitionOutStateReferences;
    static ReferenceList paramSubPlanReferences;
    static ReferenceList paramSubVarReferences;
    static ReferenceList paramVarReferences;
    static ReferenceList conditionVarReferences;
    static ReferenceList quantifierScopeReferences;
    static ReferenceList epStateReferences;
    static ReferenceList epTaskReferences;
    static ReferenceList planTypePlanReferences;
    static ReferenceList rtmRoleReferences;
    static ReferenceList charCapReferences;
    static ReferenceList charCapValReferences;
    static ReferenceList planningProblemPlanReferences;
    static ReferenceList planningProblemPlanWaitReferences;
    static ReferenceList planningProblemPlanAlternativeReferences;

    static const AlicaElement* getElement(const int64_t id);

private:
    Factory() = delete;
    static PlanRepository* planRepository;
    static std::map<int64_t, AlicaElement*>* elements;
};

template <typename T>
T Factory::getValue(const YAML::Node& node, const std::string& key)
{
    if (!node[key] || YAML::NodeType::Null == node[key].Type()) {
        AlicaEngine::abort("Factory: Node does not provide an value for: ", key);
        // does not happen, because abort() cancels the current process
        return T();
    } else {
        return node[key].as<T>();
    }
}

template <typename T>
T Factory::getValue(const YAML::Node& node, const std::string& key, T defaultValue)
{
    if (!node[key] || YAML::NodeType::Null == node[key].Type()) {
        return defaultValue;
    } else {
        return node[key].as<T>();
    }
}
} // namespace alica
