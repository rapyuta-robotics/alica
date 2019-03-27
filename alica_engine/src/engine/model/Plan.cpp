/*
 * Plan.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Plan.h"

#include <assert.h>

#include "engine/model/EntryPoint.h"
#include "engine/model/State.h"
#include "engine/model/Task.h"

namespace alica
{
Plan::Plan(int64_t id)
        : AbstractPlan(id)
        , _postCondition(nullptr)
        , _minCardinality(0)
        , _maxCardinality(0)
        , _utilityThreshold(1.0)
{
}

Plan::~Plan() {}

const EntryPoint* Plan::getEntryPointTaskID(int64_t taskID) const
{
    for (const EntryPoint* ep : _entryPoints) {
        const Task* task = ep->getTask();
        assert(task != nullptr);
        if (task->getId() == taskID) {
            return ep;
        }
    }
    return nullptr;
}

const EntryPoint* Plan::getEntryPointByID(int64_t epID) const
{
    for (const EntryPoint* ep : _entryPoints) {
        if (ep->getId() == epID) {
            return ep;
        }
    }
    return nullptr;
}

const State* Plan::getStateByID(int64_t id) const
{
    for (const State* s : _states) {
        if (s->getId() == id) {
            return s;
        }
    }
    return nullptr;
}

void Plan::setEntryPoints(const EntryPointGrp& entryPoints)
{
    _entryPoints = entryPoints;
}

void Plan::setFailureStates(const FailureStateGrp& failureStates)
{
    _failureStates = failureStates;
}

void Plan::setMaxCardinality(int maxCardinality)
{
    _maxCardinality = maxCardinality;
}

void Plan::setMinCardinality(int minCardinality)
{
    _minCardinality = minCardinality;
}

void Plan::setPostCondition(const PostCondition* postCondition)
{
    _postCondition = postCondition;
}

void Plan::setStates(const StateGrp& states)
{
    _states = states;
}

void Plan::setSuccessStates(const SuccessStateGrp& successStates)
{
    _successStates = successStates;
}

void Plan::setSyncTransitions(const SyncTransitionGrp& syncTransitions)
{
    _syncTransitions = syncTransitions;
}

void Plan::setTransitions(const TransitionGrp& transitions)
{
    _transitions = transitions;
}

std::string Plan::toString() const
{
    std::stringstream ss;
    ss << AbstractPlan::toString();
    for (const EntryPoint* ep : this->_entryPoints) {
        ss << ep->toString();
    }
    return ss.str();
}

} // namespace alica
