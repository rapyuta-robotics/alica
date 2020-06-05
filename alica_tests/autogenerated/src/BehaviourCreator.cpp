#include "BehaviourCreator.h"
#include "Behaviour/AlwaysFail.h"
#include "Behaviour/Attack.h"
#include "Behaviour/AttackOpp.h"
#include "Behaviour/ConstraintUsingBehaviour.h"
#include "Behaviour/CountIndefinitely.h"
#include "Behaviour/DefendMid.h"
#include "Behaviour/MidFieldStandard.h"
#include "Behaviour/NotToTrigger.h"
#include "Behaviour/ReadConfigurationBehaviour.h"
#include "Behaviour/SuccessSpam.h"
#include "Behaviour/Tackle.h"
#include "Behaviour/TriggerA.h"
#include "Behaviour/TriggerB.h"
#include "Behaviour/TriggerC.h"
#include "engine/BasicBehaviour.h"

namespace alica
{

BehaviourCreator::BehaviourCreator() {}

BehaviourCreator::~BehaviourCreator() {}

std::shared_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(long behaviourId)
{
    switch (behaviourId) {
    case 1402488696205:
        return std::make_shared<MidFieldStandard>();
        break;
    case 1402488730695:
        return std::make_shared<DefendMid>();
        break;
    case 1402488848841:
        return std::make_shared<Attack>();
        break;
    case 1402488939130:
        return std::make_shared<Tackle>();
        break;
    case 1402489351885:
        return std::make_shared<AttackOpp>();
        break;
    case 1414068597716:
        return std::make_shared<ConstraintUsingBehaviour>();
        break;
    case 1428508297492:
        return std::make_shared<TriggerA>();
        break;
    case 1428508316905:
        return std::make_shared<TriggerB>();
        break;
    case 1428508355209:
        return std::make_shared<TriggerC>();
        break;
    case 1429017274116:
        return std::make_shared<NotToTrigger>();
        break;
    case 1522377401286:
        return std::make_shared<SuccessSpam>();
        break;
    case 1529456643148:
        return std::make_shared<CountIndefinitely>();
        break;
    case 1532424188199:
        return std::make_shared<AlwaysFail>();
        break;
    case 1588061129360:
        return std::make_shared<ReadConfigurationBehaviour>();
        break;
    default:
        std::cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
