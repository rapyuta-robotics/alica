#include "engine/model/Transition.h"
#include "engine/IAlicaWorldModel.h"
#include "engine/RunningPlan.h"
#include "engine/blackboard/KeyMapping.h"
#include "engine/model/PostCondition.h"
#include "engine/model/TransitionCondition.h"

namespace alica
{

Transition::Transition()
        : _transitionCondition(nullptr)
        , _inState(nullptr)
        , _outState(nullptr)
        , _synchronisation(nullptr)
        , _keyMapping(nullptr)
        , _preConditionId(0)
{
}

Transition::~Transition() {}

void Transition::setTransitionCondition(TransitionCondition* transitionCondition)
{
    _transitionCondition = transitionCondition;
}

void Transition::setInState(State* inState)
{
    _inState = inState;
}

void Transition::setOutState(State* outState)
{
    _outState = outState;
}

void Transition::setSynchronisation(Synchronisation* synchronisation)
{
    _synchronisation = synchronisation;
}

} // namespace alica
