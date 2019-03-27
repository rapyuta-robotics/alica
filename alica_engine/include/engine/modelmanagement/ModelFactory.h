#pragma once

#include <list>
#include <map>
#include <memory>
#include <yaml-cpp/node/node.h>

#include "tinyxml2.h"

namespace alica
{
class PlanParser;
class PlanRepository;
class Plan;
class EntryPoint;
class AlicaElement;
class State;
class SuccessState;
class FailureState;
class Transition;
class SyncTransition;
class Parametrisation;
class PreCondition;
class PostCondition;
class RuntimeCondition;
class Quantifier;
class BehaviourConfiguration;
class Variable;
class Role;
class RoleDefinitionSet;
class CapabilityDefinitionSet;
class Characteristic;
class Capability;
class RoleTaskMapping;
class RoleSet;
class PlanningProblem;

class Parameter;

/**
 * Constructs Model elements, i.e., objects inheriting from <see cref="PlanElement"/> given their XML representation.
 */
class ModelFactory
{
public:
    ModelFactory(PlanParser* p, PlanRepository* rep);
    virtual ~ModelFactory();

    bool ignoreMasterPlanId;
    bool getIgnoreMasterPlanId();
    void setIgnoreMasterPlanId(bool value);
    Plan* createPlan(YAML::Node& node);
    std::map<int64_t, AlicaElement*>* getElements();
    void setElements(const std::map<int64_t, AlicaElement*>& elements);
    std::string getNameOfNode(tinyxml2::XMLElement* node);
    void createTasks(YAML::Node& node);
    void createBehaviour(YAML::Node& node);
    void createCapabilityDefinitionSet(tinyxml2::XMLDocument* node);
    void createRoleDefinitionSet(tinyxml2::XMLDocument* node);
    void createPlanType(YAML::Node& node);
    void computeReachabilities();
    void attachPlanReferences();
    void attachRoleReferences();
    void attachCharacteristicReferences();
    RoleSet* createRoleSet(tinyxml2::XMLDocument* node, Plan* masterPlan);
    static const EntryPoint* generateIdleEntryPoint();

private:
    static const std::string entryPoints;
    static const std::string states;
    static const std::string transitions;
    static const std::string vars;
    static const std::string synchronisations;
    static const std::string state;
    static const std::string task;
    static const std::string inTransitions;
    static const std::string outTransitions;
    static const std::string plans;
    static const std::string parametrisation;
    static const std::string subplan;
    static const std::string subvar;
    static const std::string var;
    static const std::string postCondition;
    static const std::string inState;
    static const std::string outState;
    static const std::string preCondition;
    static const std::string runtimeCondition;
    static const std::string synchronisation;
    static const std::string quantifiers;
    static const std::string sorts;
    static const std::string parameters;
    static const std::string mappings;
    static const std::string taskPriorities;
    static const std::string role;
    static const std::string capabilities;
    static const std::string capValues;
    static const std::string roles;
    static const std::string characteristics;
    static const std::string capability;
    static const std::string value;

    PlanParser* parser;
    PlanRepository* rep;
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

    void setAlicaElementAttributes(AlicaElement* ae, YAML::Node node);
    EntryPoint* createEntryPoint(const YAML::Node&  element);
    State* createState(tinyxml2::XMLElement* element);
    SuccessState* createSuccessState(tinyxml2::XMLElement* element);
    FailureState* createFailureState(tinyxml2::XMLElement* element);
    Transition* createTransition(tinyxml2::XMLElement* element, Plan* plan);
    SyncTransition* createSyncTransition(tinyxml2::XMLElement* element);
    Parameter* createParameter(tinyxml2::XMLElement* element);
    Parametrisation* createParametrisation(YAML::Node node);
    PreCondition* createPreCondition(tinyxml2::XMLElement* element);
    PostCondition* createPostCondition(tinyxml2::XMLElement* element);
    RuntimeCondition* createRuntimeCondition(tinyxml2::XMLElement* element);
    Quantifier* createQuantifier(tinyxml2::XMLElement* element);
    RoleTaskMapping* createRoleTaskMapping(tinyxml2::XMLElement* element);
    Capability* createCapability(tinyxml2::XMLElement* element);
    Role* createRole(tinyxml2::XMLElement* element);
    Characteristic* createCharacteristic(tinyxml2::XMLElement* element);
    Variable* createVariable(YAML::Node node);
    bool isReferenceNode(tinyxml2::XMLElement* node) const;
    void addElement(AlicaElement* ael);

    void createVariableTemplates();
    void removeRedundancy();
};
} // namespace alica
