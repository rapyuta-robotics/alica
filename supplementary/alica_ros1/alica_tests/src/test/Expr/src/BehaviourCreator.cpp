#include "engine/BasicBehaviour.h"
#include <alica_tests/Behaviour/AlwaysFail.h>
#include <alica_tests/Behaviour/Attack.h>
#include <alica_tests/Behaviour/AttackOpp.h>
#include <alica_tests/Behaviour/BehAAA.h>
#include <alica_tests/Behaviour/BehBAA.h>
#include <alica_tests/Behaviour/ConstraintUsingBehaviour.h>
#include <alica_tests/Behaviour/CountIndefinitely.h>
#include <alica_tests/Behaviour/DefendMid.h>
#include <alica_tests/Behaviour/EmptyBehaviour.h>
#include <alica_tests/Behaviour/MidFieldStandard.h>
#include <alica_tests/Behaviour/NotToTrigger.h>
#include <alica_tests/Behaviour/ReadConfigurationBehaviour.h>
#include <alica_tests/Behaviour/State1Behaviour.h>
#include <alica_tests/Behaviour/State2Behaviour.h>
#include <alica_tests/Behaviour/SuccessOnInitBeh.h>
#include <alica_tests/Behaviour/SuccessSpam.h>
#include <alica_tests/Behaviour/Tackle.h>
#include <alica_tests/Behaviour/TestInheritBlackboardBehaviour.h>
#include <alica_tests/Behaviour/TestParameterPassingBehaviour.h>
#include <alica_tests/Behaviour/TriggerA.h>
#include <alica_tests/Behaviour/TriggerB.h>
#include <alica_tests/Behaviour/TriggerC.h>
#include <alica_tests/BehaviourCreator.h>
#include <alica_tests/TestBehaviour.h>

namespace alica
{

BehaviourCreator::BehaviourCreator() {}

BehaviourCreator::~BehaviourCreator() {}

std::unique_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(int64_t behaviourId, BehaviourContext& context)
{
    switch (behaviourId) {
    case 1402488696205:
        return std::make_unique<MidFieldStandard>(context);
        break;
    case 1402488730695:
        return std::make_unique<DefendMid>(context);
        break;
    case 1402488848841:
        return std::make_unique<Attack>(context);
        break;
    case 1402488939130:
        return std::make_unique<Tackle>(context);
        break;
    case 1402489351885:
        return std::make_unique<AttackOpp>(context);
        break;
    case 1414068597716:
        return std::make_unique<ConstraintUsingBehaviour>(context);
        break;
    case 1428508297492:
        return std::make_unique<TriggerA>(context);
        break;
    case 1428508316905:
        return std::make_unique<TriggerB>(context);
        break;
    case 1428508355209:
        return std::make_unique<TriggerC>(context);
        break;
    case 1429017274116:
        return std::make_unique<NotToTrigger>(context);
        break;
    case 1522377401286:
        return std::make_unique<SuccessSpam>(context);
        break;
    case 1529456643148:
        return std::make_unique<CountIndefinitely>(context);
        break;
    case 1532424188199:
        return std::make_unique<AlwaysFail>(context);
        break;
    case 1588061129360:
        return std::make_unique<ReadConfigurationBehaviour>(context);
        break;
    case 1625610857563:
        return std::make_unique<EmptyBehaviour>(context);
        break;
    case 1629895901559:
        return std::make_unique<BehAAA>(context);
        break;
    case 1629895911592:
        return std::make_unique<BehBAA>(context);
        break;
    case 55178365253414982:
        return std::make_unique<TestBehaviour>(context);
        break;
    case 831400441334251600:
        return std::make_unique<TestInheritBlackboardBehaviour>(context);
        break;
    case 831400441334251602:
        return std::make_unique<TestParameterPassingBehaviour>(context);
        break;
    case 2219945377054027027:
        return std::make_unique<State2Behaviour>(context);
        break;
    case 3563417394101512880:
        return std::make_unique<State1Behaviour>(context);
        break;
    case 3821787310391665935:
        return std::make_unique<SuccessOnInitBeh>(context);
        break;
    default:
        std::cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
