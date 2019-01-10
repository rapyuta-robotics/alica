/*
 * ModelFactory.h
 *
 *  Created on: Mar 27, 2014
 *      Author: Stephan Opfer
 */

#ifndef MODELFACTORY_H_
#define MODELFACTORY_H_

#include <list>
#include <map>
#include <memory>

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
    Plan* createPlan(tinyxml2::XMLDocument* node);
    std::map<int64_t, AlicaElement*>* getElements();
    void setElements(const std::map<int64_t, AlicaElement*>& elements);
    std::string getNameOfNode(tinyxml2::XMLElement* node);
    void createTasks(tinyxml2::XMLDocument* node);
    void createBehaviour(tinyxml2::XMLDocument* node);
    void createCapabilityDefinitionSet(tinyxml2::XMLDocument* node);
    void createRoleDefinitionSet(tinyxml2::XMLDocument* node);
    void createPlanType(tinyxml2::XMLDocument* node);
    void createPlanningProblem(tinyxml2::XMLDocument* node);
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
    static const std::string conditions;
    static const std::string vars;
    static const std::string synchronisations;
    static const std::string rating;
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
    static const std::string synchronisation;
    static const std::string quantifiers;
    static const std::string sorts;
    static const std::string configurations;
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
    static const std::string waitPlan;
    static const std::string alternativePlan;

    PlanParser* _parser;
    PlanRepository* _rep;
    std::map<int64_t, AlicaElement*> _elements;

    using ReferenceList = std::list<std::pair<int64_t, int64_t>>;

    ReferenceList _stateInTransitionReferences;
    ReferenceList _stateOutTransitionReferences;
    ReferenceList _statePlanReferences;
    ReferenceList _transitionSynchReferences;
    ReferenceList _paramSubPlanReferences;
    ReferenceList _transitionAimReferences;
    ReferenceList _paramSubVarReferences;
    ReferenceList _paramVarReferences;
    ReferenceList _conditionVarReferences;
    ReferenceList _quantifierScopeReferences;
    ReferenceList _epStateReferences;
    ReferenceList _epTaskReferences;
    ReferenceList _planTypePlanReferences;
    ReferenceList _rtmRoleReferences;
    ReferenceList _charCapReferences;
    ReferenceList _charCapValReferences;
    ReferenceList _planningProblemPlanReferences;
    ReferenceList _planningProblemPlanWaitReferences;
    ReferenceList _planningProblemPlanAlternativeReferences;

    void setAlicaElementAttributes(AlicaElement* ae, tinyxml2::XMLElement* ele);
    EntryPoint* createEntryPoint(tinyxml2::XMLElement* element);
    State* createState(tinyxml2::XMLElement* element);
    SuccessState* createSuccessState(tinyxml2::XMLElement* element);
    FailureState* createFailureState(tinyxml2::XMLElement* element);
    Transition* createTransition(tinyxml2::XMLElement* element, Plan* plan);
    SyncTransition* createSyncTransition(tinyxml2::XMLElement* element);
    Parameter* createParameter(tinyxml2::XMLElement* element);
    Parametrisation* createParametrisation(tinyxml2::XMLElement* element);
    PreCondition* createPreCondition(tinyxml2::XMLElement* element);
    PostCondition* createPostCondition(tinyxml2::XMLElement* element);
    RuntimeCondition* createRuntimeCondition(tinyxml2::XMLElement* element);
    Quantifier* createQuantifier(tinyxml2::XMLElement* element);
    BehaviourConfiguration* createBehaviourConfiguration(tinyxml2::XMLElement* element);
    RoleTaskMapping* createRoleTaskMapping(tinyxml2::XMLElement* element);
    Capability* createCapability(tinyxml2::XMLElement* element);
    Role* createRole(tinyxml2::XMLElement* element);
    Characteristic* createCharacteristic(tinyxml2::XMLElement* element);
    Variable* createVariable(tinyxml2::XMLElement* element);
    bool isReferenceNode(tinyxml2::XMLElement* node) const;
    void addElement(AlicaElement* ael);

    void createVariableTemplates();
    void removeRedundancy();
};
} // namespace alica

#endif /* MODELFACTORY_H_ */
