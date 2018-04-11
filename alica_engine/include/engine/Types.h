#pragma once

#include <vector>
#include <string>
#include <unordered_map>

namespace supplementary {
class AgentID;
}

namespace alica {
class AbstractPlan;
class AlicaElement;
class BehaviourConfiguration;
class Capability;
class CapValue;
class EntryPoint;
class FailureState;
class Parameter;
class Plan;
class Quantifier;
class Role;
class RoleUsage;
class State;
class SuccessState;
class SyncTransition;
class Task;
class Transition;
class Variable;
class Parametrisation;

using AbstractPlanSet = std::vector<const AbstractPlan*>;
using AgentSet = std::vector<const supplementary::AgentID*>;
using AlicaElementSet = std::vector<const AlicaElement*>;
using BehaviourConfigurationSet = std::vector<const BehaviourConfiguration*>;
using CapabilitySet = std::vector<const Capability*>;
using CapValueSet = std::vector<const CapValue*>;
using EntryPointSet = std::vector<const EntryPoint*>;
using FailureStateSet = std::vector<const FailureState*>;
using ParameterSet = std::vector<const Parameter*>;
using ParametrisationSet = std::vector<const Parametrisation*>;
using PlanSet = std::vector<const Plan*>;
using QuantifierSet = std::vector<const Quantifier*>;
using RoleVector = std::vector<const Role*>;
using RoleUsageSet = std::vector<const RoleUsage*>;
using StateSet = std::vector<const State*>;
using SuccessStateSet = std::vector<const SuccessState*>;
using SyncTransitionSet = std::vector<const SyncTransition*>;
using TaskSet = std::vector<const Task*>;
using TransitionSet = std::vector<const Transition*>;
using VariableSet = std::vector<const Variable*>;

using BehaviourParameterMap = std::unordered_map<std::string, std::string>;
}  // namespace alica