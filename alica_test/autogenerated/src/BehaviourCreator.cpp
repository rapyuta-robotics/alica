using namespace std;

#include "BehaviourCreator.h"
#include "engine/BasicBehaviour.h"

#include  "Plans/Behaviour/ConstraintUsingBehaviour.h"

#include  "Plans/Behaviour/MidFieldStandard.h"

#include  "Plans/GSolver/SolverTestBehaviour.h"

#include  "Plans/Behaviour/AttackOpp.h"

#include  "Plans/Behaviour/Attack.h"

#include  "Plans/Behaviour/Tackle.h"

#include  "Plans/Behaviour/Joystick.h"

#include  "Plans/Behaviour/DefendMid.h"

#include  "Plans/Behaviour/NewBehaviour.h"

namespace alica
{

    BehaviourCreator::BehaviourCreator()
    {
    }

    BehaviourCreator::~BehaviourCreator()
    {
    }

    shared_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(long behaviourConfId)
    {
        switch (behaviourConfId)
        {

            case 1414068618837:

                return make_shared<ConstraintUsingBehaviour>();
                break;

            case 1402488712657:

                return make_shared<MidFieldStandard>();
                break;

            case 1417424483320:

                return make_shared<SolverTestBehaviour>();
                break;

            case 1402489366699:

                return make_shared<AttackOpp>();
                break;

            case 1402488866727:

                return make_shared<Attack>();
                break;

            case 1402488956661:

                return make_shared<Tackle>();
                break;

            case 1421854707061:

                return make_shared<Joystick>();
                break;

            case 1402488763903:

                return make_shared<DefendMid>();
                break;

            case 1413810775049:

                return make_shared<NewBehaviour>();
                break;

            default:
                cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourConfId << endl;
                throw new exception();
                break;
        }
    }
}
