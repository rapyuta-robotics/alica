using namespace std;

#include "BehaviourCreator.h"
#include "engine/BasicBehaviour.h"

#include "Plans/Behaviour/TriggerC.h"

#include "Plans/Behaviour/DefendMid.h"

#include "Plans/ProblemModule/QueryBehaviour1.h"

#include "Plans/Behaviour/NotToTrigger.h"

#include "Plans/Behaviour/SuccessSpam.h"

#include "Plans/Behaviour/AttackOpp.h"

#include "Plans/Behaviour/TriggerA.h"

#include "Plans/Behaviour/Joystick.h"

#include "Plans/Behaviour/ConstraintUsingBehaviour.h"

#include "Plans/Behaviour/NewBehaviour.h"

#include "Plans/Behaviour/Attack.h"

#include "Plans/Behaviour/TriggerB.h"

#include "Plans/Behaviour/Tackle.h"

#include "Plans/Behaviour/MidFieldStandard.h"

#include "Plans/GSolver/SolverTestBehaviour.h"

#include "Plans/Behaviour/CountIndefinitely.h"

namespace alica
{

BehaviourCreator::BehaviourCreator() {}

BehaviourCreator::~BehaviourCreator() {}

shared_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(long behaviourConfId)
{
    switch (behaviourConfId) {

    case 1428508367402:

        return make_shared<TriggerC>();
        break;

    case 1402488763903:

        return make_shared<DefendMid>();
        break;

    case 1479556115746:

        return make_shared<QueryBehaviour1>();
        break;

    case 1429017293301:

        return make_shared<NotToTrigger>();
        break;

    case 1522377419087:

        return make_shared<SuccessSpam>();
        break;

    case 1402489366699:

        return make_shared<AttackOpp>();
        break;

    case 1428508312886:

        return make_shared<TriggerA>();
        break;

    case 1421854707061:

        return make_shared<Joystick>();
        break;

    case 1414068618837:

        return make_shared<ConstraintUsingBehaviour>();
        break;

    case 1413810775049:

        return make_shared<NewBehaviour>();
        break;

    case 1402488866727:

        return make_shared<Attack>();
        break;

    case 1428508331620:

        return make_shared<TriggerB>();
        break;

    case 1402488956661:

        return make_shared<Tackle>();
        break;

    case 1402488712657:

        return make_shared<MidFieldStandard>();
        break;

    case 1417424483320:

        return make_shared<SolverTestBehaviour>();
        break;

    case 1529456686038:

        return make_shared<CountIndefinitely>();
        break;

    default:
        cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourConfId << endl;
        throw new exception();
        break;
    }
}
}
