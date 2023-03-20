#pragma once

#include "AlicaElement.h"
#include "engine/Types.h"

namespace alica
{
class Plan;
class Transition;
class AbstractPlan;
class VariableBinding;
class EntryPoint;

/**
 * A State is a plan element inhabitable by agents, which contains sub-plans, sub-plantypes, and behaviours.
 */
class State : public AlicaElement
{
public:
    enum StateType
    {
        NORMAL,
        SUCCESS,
        FAILURE
    };
    State();
    State(StateType t);

    std::string toString(std::string indent = "") const override;
    void setInPlan(const Plan* p);
    const Plan* getInPlan() const { return _inPlan; }
    void setEntryPoint(const EntryPoint* entryPoint);
    const EntryPoint* getEntryPoint() const { return _entryPoint; }
    void addConfAbstractPlanWrapper(const ConfAbstractPlanWrapper* c);
    const ConfAbstractPlanWrapperGrp& getConfAbstractPlanWrappers() const { return _confAbstractPlanWrappers; }
    void addInTransition(const Transition* t);
    const TransitionGrp& getInTransitions() const { return _inTransitions; }
    void addOutTransition(const Transition* t);
    const TransitionGrp& getOutTransitions() const { return _outTransitions; }
    void addParametrisation(const VariableBinding* v);
    const VariableBindingGrp& getParametrisation() const { return _variableBindingGrp; }

    bool isTerminal() const { return _type != NORMAL; }
    bool isSuccessState() const { return _type == SUCCESS; }
    bool isFailureState() const { return _type == FAILURE; }

private:
    /**
     * The list of AbstractPlans meant to be executed in the context of this state.
     */
    ConfAbstractPlanWrapperGrp _confAbstractPlanWrappers;
    /**
     * The list of Transitions leading to this state.
     */
    TransitionGrp _inTransitions;
    /**
     * The list ofTransitions going from this state to another one.
     */
    TransitionGrp _outTransitions;
    /**
     * The list of VariableBindings, which bind variables of sub-plans to variables in this state's plan.
     */
    VariableBindingGrp _variableBindingGrp;
    /**
     * The plan containing this state.
     */
    const Plan* _inPlan;
    /**
     * EntryPoint of the State
     */
    const EntryPoint* _entryPoint;
    /**
     * whether this is a success state, failure state or a normal state.
     */
    StateType _type;
};

} // namespace alica
