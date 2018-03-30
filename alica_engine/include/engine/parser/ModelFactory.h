/*
 * ModelFactory.h
 *
 *  Created on: Mar 27, 2014
 *      Author: Stephan Opfer
 */

#ifndef MODELFACTORY_H_
#define MODELFACTORY_H_

#include <memory>
#include <list>
#include <map>

using namespace std;
#include "tinyxml2.h"

namespace alica {
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
class ModelFactory {
public:
    ModelFactory(PlanParser* p, PlanRepository* rep);
    virtual ~ModelFactory();

    bool ignoreMasterPlanId;
    bool getIgnoreMasterPlanId();
    void setIgnoreMasterPlanId(bool value);
    Plan* createPlan(tinyxml2::XMLDocument* node);
    map<long, AlicaElement*>* getElements();
    void setElements(const map<long, AlicaElement*>& elements);
    string getNameOfNode(tinyxml2::XMLElement* node);
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

private:
    static const string entryPoints;
    static const string states;
    static const string transitions;
    static const string conditions;
    static const string vars;
    static const string synchronisations;
    static const string rating;
    static const string state;
    static const string task;
    static const string inTransitions;
    static const string outTransitions;
    static const string plans;
    static const string parametrisation;
    static const string subplan;
    static const string subvar;
    static const string var;
    static const string postCondition;
    static const string inState;
    static const string outState;
    static const string preCondition;
    static const string synchronisation;
    static const string quantifiers;
    static const string sorts;
    static const string configurations;
    static const string parameters;
    static const string mappings;
    static const string taskPriorities;
    static const string role;
    static const string capabilities;
    static const string capValues;
    static const string roles;
    static const string characteristics;
    static const string capability;
    static const string value;
    static const string waitPlan;
    static const string alternativePlan;

    PlanParser* parser;
    PlanRepository* rep;
    map<long, AlicaElement*> elements;
    list<pair<long, long>> stateInTransitionReferences;
    list<pair<long, long>> stateOutTransitionReferences;
    list<pair<long, long>> statePlanReferences;
    list<pair<long, long>> transitionSynchReferences;
    list<pair<long, long>> transitionAimReferences;
    list<pair<long, long>> paramSubPlanReferences;
    list<pair<long, long>> paramSubVarReferences;
    list<pair<long, long>> paramVarReferences;
    list<pair<long, long>> conditionVarReferences;
    list<pair<long, long>> quantifierScopeReferences;
    list<pair<long, long>> epStateReferences;
    list<pair<long, long>> epTaskReferences;
    list<pair<long, long>> planTypePlanReferences;
    list<pair<long, long>> rtmRoleReferences;
    list<pair<long, long>> charCapReferences;
    list<pair<long, long>> charCapValReferences;
    list<pair<long, long>> planningProblemPlanReferences;
    list<pair<long, long>> planningProblemPlanWaitReferences;
    list<pair<long, long>> planningProblemPlanAlternativeReferences;

    void setAlicaElementAttributes(AlicaElement* ael, tinyxml2::XMLElement* ele);
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
    bool isReferenceNode(tinyxml2::XMLElement* node);
    void addElement(AlicaElement* ael);

protected:
    void removeRedundancy();
};
}  // namespace alica

#endif /* MODELFACTORY_H_ */
