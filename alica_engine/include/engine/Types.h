#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace alica
{
class AbstractPlan;
class AlicaElement;
class Capability;
class CapValue;
class Condition;
class ConfAbstractPlanWrapper;
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
using ConfAbstractPlanWrapperGrp = std::vector<const ConfAbstractPlanWrapper*>;
using AgentGrp = std::vector<uint64_t>;
using AlicaElementGrp = std::vector<const AlicaElement*>;
using CapabilityGrp = std::vector<const Capability*>;
using CapValueGrp = std::vector<const CapValue*>;
using ConditionGrp = std::vector<const Condition*>;
using DomainVariableGrp = std::vector<const DomainVariable*>;
using EntryPointGrp = std::vector<const EntryPoint*>;
using FailureStateGrp = std::vector<const FailureState*>;
using IdGrp = std::vector<int64_t>;
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

using ParameterMap = std::unordered_map<std::string, Parameter*>;
using AgentStatePair = std::pair<uint64_t, const State*>;

} // namespace alica
