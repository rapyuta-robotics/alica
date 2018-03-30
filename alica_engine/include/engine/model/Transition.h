/*
 * Transition.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef TRANSITION_H_
#define TRANSITION_H_

#include "AlicaElement.h"
#include <memory>
#include "engine/RunningPlan.h"

namespace alica {

class State;
class SyncTransition;
class PreCondition;

/**
 * Connects two States in a Plan
 */
class Transition : public AlicaElement {
public:
    Transition();
    virtual ~Transition();
    PreCondition* getPreCondition();
    void setPreCondition(PreCondition* preCondition);
    State* getOutState();
    State* getInState();
    void setInState(State* inState);
    void setOutState(State* outState);
    SyncTransition* getSyncTransition();
    void setSyncTransition(SyncTransition* syncTransition);
    bool evalCondition(shared_ptr<RunningPlan> r);

private:
    /**
     * The condition guarding this transition.
     */
    PreCondition* preCondition;
    /**
     * The state from which this transition leads away.
     */
    State* inState;
    /**
     * The state this transition leads to
     */
    State* outState;
    /**
     * The SyncTransition this transition belongs to. Null if it does not belong to any.
     */
    SyncTransition* syncTransition;
};

}  // namespace alica

#endif /* TRANSITION_H_ */
