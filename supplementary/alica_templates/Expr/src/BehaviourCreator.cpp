#include "BehaviourCreator.h"
#include "engine/BasicBehaviour.h"

namespace alica
{

BehaviourCreator::BehaviourCreator() {}

BehaviourCreator::~BehaviourCreator() {}

std::unique_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(int64_t behaviourId, BehaviourContext& context)
{
    switch (behaviourId) {
    default:
        std::cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
