/*
 * Transition.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Transition.h"
#include "engine/RunningPlan.h"
#include "engine/model/Plan.h"
#include "engine/model/PostCondition.h"
#include "engine/model/PreCondition.h"

namespace alica
{

Transition::Transition()
        : _preCondition(nullptr)
        , _inState(nullptr)
        , _outState(nullptr)
        , _syncTransition(nullptr)
{
}

Transition::~Transition() {}

bool Transition::evalCondition(const RunningPlan& r) const
{
    return _preCondition->evaluate(r);
}

void Transition::setPreCondition(PreCondition* preCondition)
{
    _preCondition = preCondition;
}

void Transition::setInState(State* inState)
{
    _inState = inState;
}

void Transition::setOutState(State* outState)
{
    _outState = outState;
}

void Transition::setSyncTransition(SyncTransition* syncTransition)
{
    _syncTransition = syncTransition;
}

} // namespace alica
