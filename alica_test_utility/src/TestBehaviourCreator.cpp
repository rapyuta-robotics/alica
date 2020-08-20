#include "alica/test/TestBehaviourCreator.h"

namespace alica::test
{
TestBehaviourCreator::TestBehaviourCreator(IBehaviourCreator& defaultBehaviourCreator)
        : _defaultBehaviourCreator(defaultBehaviourCreator)
{
}

std::shared_ptr<alica::BasicBehaviour> TestBehaviourCreator::createBehaviour(int64_t behaviourID)
{
    auto behaviourIter = _behaviourCreateFunctions.find(behaviourID);
    if (behaviourIter == _behaviourCreateFunctions.end()) {
        return _defaultBehaviourCreator.createBehaviour(behaviourID);
    } else {
        return behaviourIter->second();
    }
}

} // namespace alica::test