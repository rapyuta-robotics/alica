#include "engine/model/Transition.h"
#include "engine/RunningPlan.h"
#include "engine/blackboard/KeyMapping.h"
#include "engine/model/PostCondition.h"
#include "engine/model/TransitionCondition.h"

namespace alica
{

Transition::Transition(std::unique_ptr<KeyMapping> keyMapping)
        : _transitionCondition(nullptr)
        , _inState(nullptr)
        , _outState(nullptr)
        , _synchronisation(nullptr)
        , _keyMapping(std::move(keyMapping))
{
}

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
