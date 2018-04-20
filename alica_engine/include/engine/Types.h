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
class State;
class SuccessState;
class SyncTransition;
class Task;
class Transition;
class Variable;
class Parametrisation;

using AbstractPlanBag = std::vector<const AbstractPlan*>;
using AgentBag = std::vector<const supplementary::AgentID*>;
using AlicaElementBag = std::vector<const AlicaElement*>;
using BehaviourConfigurationBag = std::vector<const BehaviourConfiguration*>;
using CapabilityBag = std::vector<const Capability*>;
using CapValueBag = std::vector<const CapValue*>;
using EntryPointBag = std::vector<const EntryPoint*>;
using FailureStateBag = std::vector<const FailureState*>;
using ParameterBag = std::vector<const Parameter*>;
using ParametrisationBag = std::vector<const Parametrisation*>;
using PlanBag = std::vector<const Plan*>;
using QuantifierBag = std::vector<const Quantifier*>;
using RoleBag = std::vector<const Role*>;
using StateGrp = std::vector<const State*>;
using SuccessStateGrp = std::vector<const SuccessState*>;
using SyncTransitionGrp = std::vector<const SyncTransition*>;
using TaskGrp = std::vector<const Task*>;
using TransitionGrp = std::vector<const Transition*>;
using VariableGrp = std::vector<const Variable*>;

using BehaviourParameterMap = std::unordered_map<std::string, std::string>;
}  // namespace alica
