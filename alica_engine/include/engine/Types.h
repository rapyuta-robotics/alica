#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace alica
{
class AbstractPlan;
class AgentIDConstPtr;
class AlicaElement;
class BehaviourConfiguration;
class Capability;
class CapValue;
class Condition;
class DomainVariable;
class EntryPoint;
class FailureState;
class Parameter;
class VariableBinding;
class Plan;
class Quantifier;
class Role;
class SolverVariable;
class State;
class SuccessState;
class Synchronisation;
class Task;
class Transition;
class Variable;

using AbstractPlanGrp = std::vector<const AbstractPlan*>;
using AgentGrp = std::vector<AgentIDConstPtr>;
using AlicaElementGrp = std::vector<const AlicaElement*>;
using BehaviourConfigurationGrp = std::vector<const BehaviourConfiguration*>;
using CapabilityGrp = std::vector<const Capability*>;
using CapValueGrp = std::vector<const CapValue*>;
using ConditionGrp = std::vector<const Condition*>;
using DomainVariableGrp = std::vector<const DomainVariable*>;
using EntryPointGrp = std::vector<const EntryPoint*>;
using FailureStateGrp = std::vector<const FailureState*>;
using IdGrp = std::vector<int64_t>;
using ParameterGrp = std::vector<const Parameter*>;
using VariableBindingGrp = std::vector<const VariableBinding*>;
using PlanGrp = std::vector<const Plan*>;
using QuantifierGrp = std::vector<const Quantifier*>;
using RoleGrp = std::vector<const Role*>;
using StateGrp = std::vector<const State*>;
using SuccessStateGrp = std::vector<const SuccessState*>;
using SynchronisationGrp = std::vector<const Synchronisation*>;
using TaskGrp = std::vector<const Task*>;
using TransitionGrp = std::vector<const Transition*>;
using VariableGrp = std::vector<const Variable*>;

using BehaviourParameterMap = std::unordered_map<std::string, std::string>;

using AgentStatePair = std::pair<AgentIDConstPtr, const State*>;

} // namespace alica
