#include "alica/TestBehaviourCreator.h"

namespace alica
{
TestBehaviourCreator::TestBehaviourCreator(IBehaviourCreator& defaultBehaviourCreator)
        : _defaultBehaviourCreator(defaultBehaviourCreator)
{
}

std::shared_ptr<BasicBehaviour> TestBehaviourCreator::createBehaviour(int64_t behaviourId)
{
    auto behaviourIter = _behaviourMockUps.find(behaviourId);
    if (behaviourIter == _behaviourMockUps.end()) {
        return _defaultBehaviourCreator.createBehaviour(behaviourId);
    } else {
        return behaviourIter->second;
    }
}

void TestBehaviourCreator::setBehaviourMockUp(int64_t behaviourId, const std::shared_ptr<BasicBehaviour>& behaviourMockUp)
{
    auto insertResultIter = _behaviourMockUps.insert(std::make_pair(behaviourId, behaviourMockUp));
    if (!insertResultIter.second) {
        // key already existed, so overwrite its value
        _behaviourMockUps[behaviourId] = behaviourMockUp;
    }
}
} // namespace alica