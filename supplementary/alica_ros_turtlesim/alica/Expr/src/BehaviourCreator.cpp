#include "BehaviourCreator.h"
#include "Go2RandomPosition.h"
#include "GoTo.h"
#include "engine/BasicBehaviour.h"

namespace alica
{

BehaviourCreator::BehaviourCreator() {}

BehaviourCreator::~BehaviourCreator() {}

std::unique_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(int64_t behaviourId, const Behaviour *behaviourModel, IAlicaWorldModel* wm)
{
    switch (behaviourId) {
    case 4054297592460872311:
        return std::make_unique<GoTo>(wm, behaviourModel);
        break;
    case 4085572422059465423:
        return std::make_unique<Go2RandomPosition>(wm, behaviourModel);
        break;
    default:
        std::cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
