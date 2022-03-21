#include "BehaviourCreator.h"
#include "AssignPayload.h"
#include "Behaviour/AlwaysFail.h"
#include "Behaviour/Attack.h"
#include "Behaviour/AttackOpp.h"
#include "Behaviour/BehAAA.h"
#include "Behaviour/BehBAA.h"
#include "Behaviour/ConstraintUsingBehaviour.h"
#include "Behaviour/CountIndefinitely.h"
#include "Behaviour/DefendMid.h"
#include "Behaviour/EmptyBehaviour.h"
#include "Behaviour/MidFieldStandard.h"
#include "Behaviour/NotToTrigger.h"
#include "Behaviour/ReadConfigurationBehaviour.h"
#include "Behaviour/SuccessSpam.h"
#include "Behaviour/Tackle.h"
#include "Behaviour/TriggerA.h"
#include "Behaviour/TriggerB.h"
#include "Behaviour/TriggerC.h"
#include "Drop.h"
#include "DynamicTaskBehavior.h"
#include "DynamicTaskBehaviourLD.h"
#include "FreePayload.h"
#include "NavigateToDrop.h"
#include "NavigateToPick.h"
#include "Pick.h"
#include "TestBehaviour.h"
#include "TestParameterPassingBehaviour.h"
#include "engine/BasicBehaviour.h"
#include "engine/IAlicaWorldModel.h"

namespace alica
{

BehaviourCreator::BehaviourCreator() {}

BehaviourCreator::~BehaviourCreator() {}

std::shared_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(int64_t behaviourId, IAlicaWorldModel* wm)
{
    switch (behaviourId) {
    case 1402488696205:
        return std::make_shared<MidFieldStandard>(wm);
        break;
    case 1402488730695:
        return std::make_shared<DefendMid>(wm);
        break;
    case 1402488848841:
        return std::make_shared<Attack>(wm);
        break;
    case 1402488939130:
        return std::make_shared<Tackle>(wm);
        break;
    case 1402489351885:
        return std::make_shared<AttackOpp>(wm);
        break;
    case 1414068597716:
        return std::make_shared<ConstraintUsingBehaviour>(wm);
        break;
    case 1428508297492:
        return std::make_shared<TriggerA>(wm);
        break;
    case 1428508316905:
        return std::make_shared<TriggerB>(wm);
        break;
    case 1428508355209:
        return std::make_shared<TriggerC>(wm);
        break;
    case 1429017274116:
        return std::make_shared<NotToTrigger>(wm);
        break;
    case 1522377401286:
        return std::make_shared<SuccessSpam>(wm);
        break;
    case 1529456643148:
        return std::make_shared<CountIndefinitely>(wm);
        break;
    case 1532424188199:
        return std::make_shared<AlwaysFail>(wm);
        break;
    case 1588061129360:
        return std::make_shared<ReadConfigurationBehaviour>(wm);
        break;
    case 1625610857563:
        return std::make_shared<EmptyBehaviour>(wm);
        break;
    case 1629895901559:
        return std::make_shared<BehAAA>(wm);
        break;
    case 1629895911592:
        return std::make_shared<BehBAA>(wm);
        break;
    case 19516698765703926:
        return std::make_shared<DynamicTaskBehaviourLD>(wm);
        break;
    case 55178365253414982:
        return std::make_shared<TestBehaviour>(wm);
        break;
    case 422054015709952219:
        return std::make_shared<FreePayload>(wm);
        break;
    case 831400441334251602:
        return std::make_shared<TestParameterPassingBehaviour>(wm);
        break;
    case 2580816776008671737:
        return std::make_shared<Pick>(wm);
        break;
    case 3009473645416620380:
        return std::make_shared<Drop>(wm);
        break;
    case 3826644292150922713:
        return std::make_shared<AssignPayload>(wm);
        break;
    case 4044546549214673470:
        return std::make_shared<DynamicTaskBehavior>(wm);
        break;
    case 4459885370764933844:
        return std::make_shared<NavigateToDrop>(wm);
        break;
    case 4505472195947429717:
        return std::make_shared<NavigateToPick>(wm);
        break;
    default:
        std::cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
