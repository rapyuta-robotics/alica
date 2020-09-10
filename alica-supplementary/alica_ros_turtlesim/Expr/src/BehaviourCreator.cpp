#include "BehaviourCreator.h"
#include "Behaviours/Go2RandomPosition.h"
#include "Behaviours/GoTo.h"
#include "engine/BasicBehaviour.h"

namespace alica
{

BehaviourCreator::BehaviourCreator() {}

BehaviourCreator::~BehaviourCreator() {}

std::shared_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(long behaviourId)
{
    switch (behaviourId) {
    case 1542881969548:
        return std::make_shared<Go2RandomPosition>();
        break;
    case 1544160969061:
        return std::make_shared<GoTo>();
        break;
    default:
        std::cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
