#include "alica/test/TestBehaviourCreator.h"
#include <engine/BasicBehaviour.h>

namespace alica::test
{

TestBehaviourCreator::TestBehaviourCreator(std::unique_ptr<IBehaviourCreator> defaultBehaviourCreator)
        : _defaultBehaviourCreator(std::move(defaultBehaviourCreator))
{
}

std::unique_ptr<alica::BasicBehaviour> TestBehaviourCreator::createBehaviour(int64_t behaviourID, alica::BehaviourContext& context)
{
    auto behaviourIter = _behaviourCreateFunctions.find(behaviourID);
    if (behaviourIter == _behaviourCreateFunctions.end()) {
        return _defaultBehaviourCreator->createBehaviour(behaviourID, context);
    } else {
        return behaviourIter->second();
    }
}

} // namespace alica::test
