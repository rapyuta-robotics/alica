using namespace std;

#include "BehaviourCreator.h"
#include "engine/BasicBehaviour.h"

#include "Plans/ProblemModule/QueryBehaviour1.h"

#include "Plans/GSolver/SolverTestBehaviour.h"

namespace alica
{

BehaviourCreator::BehaviourCreator() {}

BehaviourCreator::~BehaviourCreator() {}

shared_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(long behaviourConfId)
{
    switch (behaviourConfId) {

    case 1479556115746:

        return make_shared<QueryBehaviour1>();
        break;

    case 1417424483320:

        return make_shared<SolverTestBehaviour>();
        break;

    default:
        cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourConfId << endl;
        throw new exception();
        break;
    }
}
} // namespace alica
