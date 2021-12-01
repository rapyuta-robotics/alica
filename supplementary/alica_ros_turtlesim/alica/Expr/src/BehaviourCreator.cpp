#include "BehaviourCreator.h"
#include "Go2RandomPosition.h"
#include "GoTo.h"
#include "engine/BasicBehaviour.h"
#include "engine/IAlicaWorldModel.h"

namespace alica
{

BehaviourCreator::BehaviourCreator() {}

BehaviourCreator::~BehaviourCreator() {}

std::shared_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(int64_t behaviourId, IAlicaWorldModel* wm)
{
    switch (behaviourId) {
    case 4054297592460872311:
        return std::make_shared<GoTo>(wm);
        break;
    case 4085572422059465423:
        return std::make_shared<Go2RandomPosition>(wm);
        break;
    default:
        std::cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
