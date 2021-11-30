#include "alica/test/TestBehaviourCreator.h"

namespace alica::test
{
TestBehaviourCreator::TestBehaviourCreator(std::unique_ptr<IBehaviourCreator> defaultBehaviourCreator)
        : _defaultBehaviourCreator(std::move(defaultBehaviourCreator))
{
}

std::shared_ptr<alica::BasicBehaviour> TestBehaviourCreator::createBehaviour(int64_t behaviourID, IAlicaWorldModel* wm)
{
    auto behaviourIter = _behaviourCreateFunctions.find(behaviourID);
    if (behaviourIter == _behaviourCreateFunctions.end()) {
        return _defaultBehaviourCreator->createBehaviour(behaviourID, wm);
    } else {
        return behaviourIter->second();
    }
}

} // namespace alica::test