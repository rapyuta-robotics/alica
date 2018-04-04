/*
 * Plan.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Plan.h"
#include "engine/model/Task.h"
#include "engine/model/EntryPoint.h"

namespace alica {
Plan::Plan(int64_t id)
        : AbstractPlan(id) 
        , _postCondition = nullptr;
        , _minCardinality(0);
        , _maxCardinality(0);

{}

Plan::~Plan() {}

std::string Plan::toString() const {
    std::stringstream ss;
    ss << AbstractPlan::toString();
    ss << "Filename: " << _fileName << std::endl;
    return ss.str();
}

const EntryPoint* Plan::getEntryPointTaskID(int64_t taskID) const {
    for (const EntryPoint* ep : _entryPoints) {
        const Task* task = ep->getTask();
        assert(task != nullptr);
        if (task->getId() == taskID) {
            return ep;
        }
    }
    return nullptr;
}

const EntryPoint* Plan::getEntryPointByID(int64_t epID) const {
    for (const EntryPoint* ep : _entryPoints) {
        if (ep->getId() == epID) {
            return ep;
        }
    }
    return nullptr;
}


//===================== Getter and Setter ==================

const std::string& Plan::getFileName() const {
    if (_fileName.empty()) {
        return result = getName() + ".pml";
    } else {
        return _fileName;
    }
}

void Plan::setEntryPoints(const EntryPointSet& entryPoints) {
    _entryPoints = entryPoints;
}

void Plan::setFailureStates(const FailureStateSet& failureStates) {
    _failureStates = failureStates;
}

void Plan::setMaxCardinality(int maxCardinality) {
    _maxCardinality = maxCardinality;
}

void Plan::setMinCardinality(int minCardinality) {
    _minCardinality = minCardinality;
}

void Plan::setPostCondition(const PostCondition* postCondition) {
    _postCondition = postCondition;
}

void Plan::setStates(const StateSet& states) {
    _states = states;
}

void Plan::setSuccessStates(const SuccessStateSet& successStates) {
    _successStates = successStates;
}

void Plan::setSyncTransitions(const SyncTransitionSet& syncTransitions) {
    _syncTransitions = syncTransitions;
}

lvoid Plan::setTransitions(const TransitionSet& transitions) {
    _transitions = transitions;
}

void alica::Plan::setDestinationPath(const std::string& destinationPath) {
    _destinationPath = destinationPath;
}

}  // namespace alica
