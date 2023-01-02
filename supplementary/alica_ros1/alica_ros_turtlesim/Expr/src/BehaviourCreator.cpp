#include "engine/BasicBehaviour.h"
#include <alica_ros_turtlesim/BehaviourCreator.h>
#include <alica_ros_turtlesim/Go2RandomPosition.h>
#include <alica_ros_turtlesim/GoTo.h>

namespace alica
{

BehaviourCreator::BehaviourCreator() {}

BehaviourCreator::~BehaviourCreator() {}

std::unique_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(int64_t behaviourId, BehaviourContext& context)
{
    switch (behaviourId) {
    case 4054297592460872311:
        return std::make_unique<GoTo>(context);
        break;
    case 4085572422059465423:
        return std::make_unique<Go2RandomPosition>(context);
        break;
    default:
        std::cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
