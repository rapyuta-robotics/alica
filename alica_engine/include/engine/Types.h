#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace alica
{
class AbstractPlan;
class AlicaElement;
class Blackboard;
class Capability;
class CapValue;
class Condition;
class ConfAbstractPlanWrapper;
class DomainVariable;
class EntryPoint;
class FailureState;
class Blackboard;
class Parameter;
class VariableBinding;
class Plan;
class Quantifier;
class Role;
class RunningPlan;
class SolverVariable;
class State;
class SuccessState;
class Synchronisation;
class Task;
class Transition;
class TransitionCondition;
class Variable;

using AbstractPlanGrp = std::vector<const AbstractPlan*>;
using ConfAbstractPlanWrapperGrp = std::vector<const ConfAbstractPlanWrapper*>;
using AgentId = uint64_t;
using AgentGrp = std::vector<AgentId>;
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
using TransitionConditionGrp = std::vector<const TransitionCondition*>;
using VariableGrp = std::vector<const Variable*>;

using ParameterMap = std::unordered_map<std::string, Parameter*>;
using AgentStatePair = std::pair<AgentId, const State*>;
using TransitionConditionCallback = std::function<bool(const Blackboard*, const RunningPlan*, const Blackboard*)>;

constexpr auto InvalidAgentID = std::numeric_limits<uint64_t>::max();

} // namespace alica
