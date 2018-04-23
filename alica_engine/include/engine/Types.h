#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace supplementary
{
class AgentID;
}

namespace alica
{
class AbstractPlan;
class AlicaElement;
class BehaviourConfiguration;
class Capability;
class CapValue;
class Condition;
class DomainVariable;
class EntryPoint;
class FailureState;
class Parameter;
class Plan;
class Quantifier;
class Role;
class State;
class SuccessState;
class SyncTransition;
class Task;
class Transition;
class Variable;
class Parametrisation;

using AbstractPlanGrp = std::vector<const AbstractPlan*>;
using AgentGrp = std::vector<const supplementary::AgentID*>;
using AlicaElementGrp = std::vector<const AlicaElement*>;
using BehaviourConfigurationGrp = std::vector<const BehaviourConfiguration*>;
using CapabilityGrp = std::vector<const Capability*>;
using CapValueGrp = std::vector<const CapValue*>;
using ConditionSet = std::vector<const Condition*>;
using DomainVariableGrp = std::vector<const DomainVariable*>;
using EntryPointGrp = std::vector<const EntryPoint*>;
using FailureStateGrp = std::vector<const FailureState*>;
using ParameterGrp = std::vector<const Parameter*>;
using ParametrisationGrp = std::vector<const Parametrisation*>;
using PlanGrp = std::vector<const Plan*>;
using QuantifierGrp = std::vector<const Quantifier*>;
using RoleGrp = std::vector<const Role*>;
using StateGrp = std::vector<const State*>;
using SuccessStateGrp = std::vector<const SuccessState*>;
using SyncTransitionGrp = std::vector<const SyncTransition*>;
using TaskSet = std::vector<const Task*>;
using TransitionGrp = std::vector<const Transition*>;
using VariableGrp = std::vector<const Variable*>;

using BehaviourParameterMap = std::unordered_map<std::string, std::string>;
} // namespace alica
