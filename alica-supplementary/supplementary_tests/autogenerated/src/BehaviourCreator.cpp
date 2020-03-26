#include "BehaviourCreator.h"
#include "GSolver/SolverTestBehaviour.h"
#include "ProblemModule/QueryBehaviour1.h"
#include "engine/BasicBehaviour.h"

namespace alica
{

BehaviourCreator::BehaviourCreator() {}

BehaviourCreator::~BehaviourCreator() {}

std::shared_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(long behaviourId)
{
    switch (behaviourId) {
    case 1417424455986:
        return std::make_shared<SolverTestBehaviour>();
        break;
    case 1479556104511:
        return std::make_shared<QueryBehaviour1>();
        break;
    default:
        std::cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
