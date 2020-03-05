using namespace std;

#include "BehaviourCreator.h"
#include "engine/BasicBehaviour.h"

#include "Plans/Behaviours/Go2RandomPosition.h"

#include "Plans/Behaviours/GoTo.h"

namespace alica {

BehaviourCreator::BehaviourCreator() {}

BehaviourCreator::~BehaviourCreator() {}

shared_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(long behaviourConfId) {
    switch (behaviourConfId) {
        case 1542881996304:

            return make_shared<Go2RandomPosition>();
            break;

        case 1544160989370:

            return make_shared<GoTo>();
            break;

        default:
            cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourConfId << endl;
            throw new exception();
            break;
    }
}
}  // namespace alica
