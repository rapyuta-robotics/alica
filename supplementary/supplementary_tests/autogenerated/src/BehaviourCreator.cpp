#include "BehaviourCreator.h"
#include "GSolver/SolverTestBehaviour.h"
#include "ProblemModule/QueryBehaviour1.h"
#include "engine/BasicBehaviour.h"
#include "engine/IAlicaWorldModel.h"

namespace alica
{

BehaviourCreator::BehaviourCreator() {}

BehaviourCreator::~BehaviourCreator() {}

std::shared_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(int64_t behaviourId, IAlicaWorldModel* wm)
{
    switch (behaviourId) {
    case 1417424455986:
        return std::make_shared<SolverTestBehaviour>(wm);
        break;
    case 1479556104511:
        return std::make_shared<QueryBehaviour1>(wm);
        break;
    default:
        std::cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
