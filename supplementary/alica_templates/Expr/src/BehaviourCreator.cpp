#include "BehaviourCreator.h"
#include "engine/BasicBehaviour.h"
#include "engine/IAlicaWorldModel.h"

namespace alica
{

BehaviourCreator::BehaviourCreator() {}

BehaviourCreator::~BehaviourCreator() {}

std::shared_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(int64_t behaviourId, IAlicaWorldModel* wm)
{
    switch (behaviourId) {
    default:
        std::cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
