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
State::State()
        : AlicaElement(0)
        , _type(Normal)
        , _inPlan(nullptr)
        , _entryPoint(nullptr) {}

State::State(StateType t)
        : AlicaElement(0)
        , _type(t)
        , _inPlan(nullptr)
        , _entryPoint(nullptr) {}

/**
 * Constructor which accepts a unique id.
 * @param id A int
 */
State::State(int64_t id)
        : AlicaElement(id)
        , _type(Normal)
        , _inPlan(nullptr)
        , _entryPoint(nullptr) {}

State::~State() {}

void State::setInPlan(const Plan* inPlan) {
    _inPlan = inPlan;
}

void State::setInTransitions(const TransitionSet& inTransitions) {
    _inTransitions = inTransitions;
}

void State::setOutTransitions(const TransitionSet& outTransition) {
    _outTransitions = outTransition;
}

void State::setParametrisation(const ParametrisationSet& parametrisation) {
    _parametrisation = parametrisation;
}

void State::setPlans(const AbstractPlanSet& plans) {
    _plans = plans;
}

void alica::State::setEntryPoint(const EntryPoint* entryPoint) {
    _entryPoint = entryPoint;
}

}  // namespace alica
