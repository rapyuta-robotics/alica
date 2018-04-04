#pragma once

#include <vector>


namespace supplementary {
    class AgentID;
}

namespace alica {
    class AbstractPlan;
    class EntryPoint;
    class FailureState;
    class Parameter;
    class Plan;
    class Quantifier;
    class State;
    class SuccessState;
    class SyncTransition;
    class Task;
    class Transition;
    class Variable;

    using AgentSet = std::vector<const supplementary::AgentID*>;

    using AbstractPlanSet = std::vector<const AbstractPlan*>;
    using EntryPointSet = std::vector<const EntryPoint*>;
    using FailureStateSet = std::vector<const FailureState*>;
    using ParameterSet = std::vector<const Parameter*>;
    using PlanSet = std::vector<const Plan*>;
    using QuantifierSet = std::vector<const Quantifier*>;
    using StateSet = std::vector<const State*>;
    using SuccessStateSet = std::vector<const SuccessState*>;
    using SyncTransitionSet = std::vector<const SyncTransition*>;
    using TaskSet = std::vector<const Task*>;
    using TransitionSet = std::vector<const Transition*>;    
    using VariableSet = std::vector<const Variable*>;


}