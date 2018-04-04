/*
 * State.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef STATE_H_
#define STATE_H_

#include <stdio.h>
#include <list>

#include "AlicaElement.h"
#include "engine/Types.h"

namespace alica {
class Plan;
class Transition;
class AbstractPlan;
class Parametrisation;
class EntryPoint;
class ModelFactory;

/**
 * A State is a plan element inhabitable by agents, which contains sub-plans, sub-plantypes, and behaviours.
 */
class State : public AlicaElement {
public:
    State();
    State(int64_t id);
    virtual ~State();
    virtual std::string toString() const;

    const Plan* getInPlan() const {return _inPlan;}
    const EntryPoint* getEntryPoint() const {return _entryPoint;}
    const AbstractPlanSet& getPlans() const {return _plans;}
    const TransitionSet& getInTransitions() const {return _inTransitions;}
    const TransitionSet& getOutTransitions() const {return _outTransitions;}
    const std::vector<const Parametrisation*>& getParametrisation() {return _parametrisation;}

    bool isTerminal() const {return _terminal;}
    bool isSuccessState() const {return _successState;}
    bool isFailureState() const {return _failureState;}

private:
    friend ModelFactory;
    void setInPlan(Plan* inPlan);
    
    void setInTransitions(const list<Transition*>& inTransitions);
    
    void setOutTransitions(list<Transition*> outTransition);
    
    void setParametrisation(const list<Parametrisation*>& parametrisation);
    
    void setPlans(const AbstractPlanSet& plans);
    
    void setSuccessState(bool successState);
    void setFailureState(bool failureState);

    void setTerminal(bool terminal);
    
    void setEntryPoint(EntryPoint* entryPoint);


    /**
     * The list of AbstractPlans meant to be executed in the context of this state.
     */
    AbstractPlanSet _plans;
    /**
     * The list of Transitions leading to this state.
     */
    TransitionSet _inTransitions;
    /**
     * The list ofTransitions going from this state to another one.
     */
    TransitionSet _outTransitions;
    /**
     * The list of Parametrisations, which bind variables of sub-plans to variables in this state's plan.
     */
    std::vector<const Parametrisation*> _parametrisation;
    /**
     * The plan containing this state.
     */
    const Plan* _inPlan;
    /**
     * EntryPoint of the State
     */
    const EntryPoint* _entryPoint;
    /**
     * whether or not this is a terminal state.
     */
    bool _terminal;
    /**
     * whether or not this is a FailureState, used to avoid casting and type checking during runtime.
     */
    bool _failureState;
    /**
     * whether or not this is a SuccessState, used to avoid casting and type checking during runtime.
     */
    bool _successState;
};

}  // namespace alica

#endif /* STATE_H_ */
