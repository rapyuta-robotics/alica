#include "engine/BasicBehaviour.h"
#include <supplementary_tests/BehaviourCreator.h>
#include <supplementary_tests/GSolver/SolverTestBehaviour.h>
#include <supplementary_tests/ProblemModule/QueryBehaviour1.h>

namespace alica
{

BehaviourCreator::BehaviourCreator() {}

BehaviourCreator::~BehaviourCreator() {}

std::unique_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(int64_t behaviourId, BehaviourContext& context)
{
    switch (behaviourId) {
    case 1417424455986:
        return std::make_unique<SolverTestBehaviour>(context);
        break;
    case 1479556104511:
        return std::make_unique<QueryBehaviour1>(context);
        break;
    default:
        std::cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
