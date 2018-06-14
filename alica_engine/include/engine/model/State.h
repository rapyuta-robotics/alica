/*
 * State.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef STATE_H_
#define STATE_H_

#include <list>
#include <stdio.h>

#include "AlicaElement.h"
#include "engine/Types.h"

namespace alica
{
class Plan;
class Transition;
class AbstractPlan;
class Parametrisation;
class EntryPoint;
class ModelFactory;

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
    State(int64_t id);
    virtual ~State();

    const Plan* getInPlan() const { return _inPlan; }
    const EntryPoint* getEntryPoint() const { return _entryPoint; }
    const AbstractPlanGrp& getPlans() const { return _plans; }
    const TransitionGrp& getInTransitions() const { return _inTransitions; }
    const TransitionGrp& getOutTransitions() const { return _outTransitions; }
    const ParametrisationGrp& getParametrisation() const { return _parametrisation; }

    bool isTerminal() const { return _type != NORMAL; }
    bool isSuccessState() const { return _type == SUCCESS; }
    bool isFailureState() const { return _type == FAILURE; }

private:
    friend ModelFactory;
    void setInPlan(const Plan* inPlan);

    void setInTransitions(const TransitionGrp& inTransitions);
    void setOutTransitions(const TransitionGrp& outTransition);

    void setParametrisation(const ParametrisationGrp& parametrisation);

    void setPlans(const AbstractPlanGrp& plans);

    void setSuccessState(bool successState);
    void setFailureState(bool failureState);

    void setTerminal(bool terminal);

    /**
     * The list of AbstractPlans meant to be executed in the context of this state.
     */
    AbstractPlanGrp _plans;
    /**
     * The list of Transitions leading to this state.
     */
    TransitionGrp _inTransitions;
    /**
     * The list ofTransitions going from this state to another one.
     */
    TransitionGrp _outTransitions;
    /**
     * The list of Parametrisations, which bind variables of sub-plans to variables in this state's plan.
     */
    ParametrisationGrp _parametrisation;
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

#endif /* STATE_H_ */
