/*
 * State.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/State.h"
namespace alica {

/**
 * Basic constructor
 */
State::State() {
    : _id(0)
    , _terminal(false)
    , _successState(false)
    , _failureState(false)
    , _inPlan(nullptr)
    , _entryPoint(nullptr)
{}

/**
 * Constructor which accepts a unique id.
 * @param id A int
 */
State::State(int64_t id) 
    : _id(id)
    , _terminal(false)
    , _successState(false)
    , _failureState(false)
    , _inPlan(nullptr)
    , _entryPoint(nullptr)
{}

State::~State() {}


void State::setFailureState(bool failureState) {
    _failureState = failureState;
}


void State::setInPlan(const Plan* inPlan) {
    _inPlan = inPlan;
}


void State::setInTransitions(const TransitionSet& inTransitions) {
    _inTransitions = inTransitions;
}

void State::setOutTransitions(const TransitionSet& outTransition) {
    _outTransitions = outTransition;
}

void State::setParametrisation(const std::vector<const Parametrisation*>& parametrisation) {
    _parametrisation = parametrisation;
}

void State::setPlans(const AbstractPlanSet& plans) {
    _plans = plans;
}

void State::setSuccessState(bool successState) {
    _successState = successState;
}

void State::setTerminal(bool terminal) {
    _terminal = terminal;
}

void alica::State::setEntryPoint(EntryPoint* entryPoint) {
    _entryPoint = entryPoint;
}

std::string State::toString() const {
    std::stringstream ss;
    ss << AlicaElement::toString();
    return ss.str();
}

}  // namespace alica
