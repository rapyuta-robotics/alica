/*
 * Plan.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef PLAN_H_
#define PLAN_H_

#include <stddef.h>
#include <string>
#include <list>

#include "AbstractPlan.h"
#include "engine/Types.h"

namespace alica {

class EntryPoint;
class FailureState;
class SuccessState;
class PostCondition;
class State;
class SyncTransition;
class Transition;
class ModelFactory;
class ExpressionHandler;
/**
 * An ALICA plan
 */
class Plan : public AbstractPlan {
public:
    Plan(int64_t id = 0);
    virtual ~Plan();

    const EntryPoint* getEntryPointTaskID(int64_t taskID) const;
    const EntryPoint* getEntryPointByID(int64_t epID) const;

    const EntryPointSet& getEntryPoints() const { return _entryPoints; }

    const StateSet& getStates() const { return _states; }
    const FailureStateSet& getFailureStates() const { return _failureStates; }
    const SuccessStateSet& getSuccessStates() const { return _successStates; }

    int getMaxCardinality() const { return _maxCardinality; }
    int getMinCardinality() const { return _minCardinality; }

    const PostCondition* getPostCondition() const { return _postCondition; }

    const TransitionSet& getTransitions() const { return _transitions; }
    const SyncTransitionSet& getSyncTransitions() const { return _syncTransitions; }



private:
    friend ModelFactory;
    friend ExpressionHandler;  // TODO: get rid of this
    void setEntryPoints(const EntryPointSet& entryPoints);
    void setFailureStates(const FailureStateSet& failurePoints);
    void setSuccessStates(const SuccessStateSet& succesPoints);
    void setMaxCardinality(int maxCardinality);
    void setMinCardinality(int minCardinality);
    void setPostCondition(const PostCondition* postCondition);
    void setStates(const StateSet& states);
    void setSyncTransitions(const SyncTransitionSet& syncTransitions);
    void setTransitions(const TransitionSet& transitions);

    int _minCardinality;
    int _maxCardinality;
    EntryPointSet _entryPoints;
    StateSet _states;
    SuccessStateSet _successStates;
    FailureStateSet _failureStates;
    SyncTransitionSet _syncTransitions;
    TransitionSet _transitions;
    const PostCondition* _postCondition;
};

}  // namespace alica

#endif /* PLAN_H_ */
