#include "alica/AlicaTestBehaviourCreator.h"

namespace alica
{
AlicaTestBehaviourCreator::AlicaTestBehaviourCreator(IBehaviourCreator& defaultBehaviourCreator)
        : _defaultBehaviourCreator(defaultBehaviourCreator)
{
}

std::shared_ptr<BasicBehaviour> AlicaTestBehaviourCreator::createBehaviour(int64_t behaviourID)
{
    auto behaviourIter = _createdBehaviours.find(behaviourID);
    if (behaviourIter == _createdBehaviours.end()) {
        std::shared_ptr<BasicBehaviour> defaultBehaviour = _defaultBehaviourCreator.createBehaviour(behaviourID);
        _createdBehaviours.insert(std::make_pair(behaviourID, defaultBehaviour));
        return defaultBehaviour;
    } else {
        return behaviourIter->second;
    }
}

void AlicaTestBehaviourCreator::setBehaviourMockUp(int64_t behaviourID, const std::shared_ptr<BasicBehaviour>& behaviourMockUp)
{
    auto insertResultIter = _createdBehaviours.insert(std::make_pair(behaviourID, behaviourMockUp));
    if (!insertResultIter.second) {
        // key already existed, so overwrite its value
        _createdBehaviours[behaviourID] = behaviourMockUp;
    }
}
} // namespace alica