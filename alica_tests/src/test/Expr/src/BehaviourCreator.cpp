#include "BehaviourCreator.h"
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
#include "TestBehaviour.h"
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
    case 55178365253414982:
        return std::make_shared<TestBehaviour>(wm);
        break;
    default:
        std::cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
