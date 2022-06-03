#include <alica_tests/conditions/conditions.h>

#include <engine/IAlicaWorldModel.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>
#include <engine/BasicPlan.h>
#include <alica_tests/TestWorldModel.h>
#include <alica_tests/SimpleSwitches.h>
#include <alica_tests/test_sched_world_model.h>
#include <alica_tests/CounterClass.h>
#include <iostream>

namespace alica
{
bool condition2872265442510628524(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1402488519140) ENABLED START*/
    // static_assert(false, "Condition 1402488519140 with name MISSING_NAME is not yet implemented");
    return true;
    /*PROTECTED REGION END*/
}
 bool condition3828316183435191952(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1402488520968) ENABLED START*/
    // static_assert(false, "Condition 1402488520968 with name MISSING_NAME is not yet implemented");
    return false;
    /*PROTECTED REGION END*/
}
bool condition2205566100638019970(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1402488558741) ENABLED START*/
    auto* worldModel = dynamic_cast<const alica_test::SchedWM*>(wm);
    return worldModel->transitionToExecuteBehaviour;
    /*PROTECTED REGION END*/
}
bool condition2901825906319407673(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1409218319990) ENABLED START*/
    // static_assert(false, "Condition 1409218319990 with name MISSING_NAME is not yet implemented");
    LockedBlackboardRO bb(*input);
    return CounterClass::called == bb.get<int>("numberOfCalls");
    /*PROTECTED REGION END*/
}
bool condition2163654295690873706(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1402489460549) ENABLED START*/
    // static_assert(false, "Condition 1402489460549 with name MISSING_NAME is not yet implemented");
    // counter class called
    return false;
    /*PROTECTED REGION END*/
}
bool condition4281647834169813432(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1402489462088) ENABLED START*/
    std::string value;
    rp->getParameter("TestValue", value);
    return value.compare("1") == 0;
    /*PROTECTED REGION END*/
}
bool condition3684268241099966909(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1402489258509) ENABLED START*/
    std::string value;
    rp->getParameter("TestValue", value);
    return value.compare("2") == 0;
    /*PROTECTED REGION END*/
}
bool condition1013158988206959873(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1402489278408) ENABLED START*/
    // static_assert(false, "Condition 1402489278408 with name MISSING_NAME is not yet implemented");
    return CounterClass::called == 4;
    /*PROTECTED REGION END*/
}
bool condition1678986049909129132(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1402500844446) ENABLED START*/
    // static_assert(false, "Condition 1402500844446 with name MISSING_NAME is not yet implemented");
    return false;
    /*PROTECTED REGION END*/
}
bool condition19871606597697646(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1402489174338) ENABLED START*/
    // static_assert(false, "Condition 1402489174338 with name MISSING_NAME is not yet implemented");
    auto worldmodel = dynamic_cast<const alicaTests::TestWorldModel*>(wm);
    return worldmodel->isTransitionCondition1747408236004727286();
    /*PROTECTED REGION END*/
}
bool condition4244459279660861567(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1402489206278) ENABLED START*/
    auto* worldModel = dynamic_cast<const alica_test::SchedWM*>(wm);
    return worldModel->transitionToExecuteBehaviourInSubPlan;
    /*PROTECTED REGION END*/
}
bool condition1770682125085719690(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1402489218027) ENABLED START*/
    const auto twm = rp->getOwnID() == 8 ? alicaTests::TestWorldModel::getOne() : alicaTests::TestWorldModel::getTwo();
    return twm->isTransitionCondition1023566846009251524();
    /*PROTECTED REGION END*/
}
bool condition190171326790683374(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1402488991641) ENABLED START*/
    const auto twm = rp->getOwnID() == 8 ? alicaTests::TestWorldModel::getOne() : alicaTests::TestWorldModel::getTwo();
    if (twm->transitionCondition3194919312481305139Enabled()) {
        return rp->isAnyChildStatus(PlanStatus::Failed);
    }
    return false;
    /*PROTECTED REGION END*/
}
bool condition2171152220550556375(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1402488993122) ENABLED START*/
    AgentId agentID8 = 8;
    if (rp->getOwnID() == agentID8) {
        return alicaTests::TestWorldModel::getOne()->isTransitionCondition1418825427317();
    } else {
        return alicaTests::TestWorldModel::getTwo()->isTransitionCondition1418825427317();
    }
    /*PROTECTED REGION END*/
}
bool condition2711102114821139213(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1402489065962) ENABLED START*/
    return CounterClass::called == 8;
    /*PROTECTED REGION END*/
}
bool condition1291995818541962959(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1402489073613) ENABLED START*/
    const auto twm = rp->getOwnID() == 8 ? alicaTests::TestWorldModel::getOne() : alicaTests::TestWorldModel::getTwo();
    return twm->isTransitionCondition1446293122737278544();
    /*PROTECTED REGION END*/
}
bool condition2208457928613785430(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1412761926856) ENABLED START*/
    // static_assert(false, "Condition 1412761926856 with name 1412761926856 is not yet implemented");
    AgentId agentID8 = 8;

    if (rp->getOwnID() == agentID8) {
        return alicaTests::TestWorldModel::getOne()->isTransitionCondition1413201227586();
    } else {
        return alicaTests::TestWorldModel::getTwo()->isTransitionCondition1413201227586();
    }
    /*PROTECTED REGION END*/
}
bool condition843443485857038179(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1413201227586) ENABLED START*/
    LockedBlackboardRO bb(*input);
    return rp->isAnyChildStatus(bb.get<PlanStatus>("childStatus"));
    /*PROTECTED REGION END*/
}
bool condition711536493236439192(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1413201389955) ENABLED START*/
    return rp->isAnyChildStatus(PlanStatus::Failed);
    /*PROTECTED REGION END*/
}
bool condition3604374027783683696(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1413201052549) ENABLED START*/
    // static_assert(false, "Condition 1413201052549 with name 1413201052549 is not yet implemented");
    return false;
    /*PROTECTED REGION END*/
}
bool condition330238006348384830(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1413201367990) ENABLED START*/
    // static_assert(false, "Condition 1413201367990 with name 1413201367990 is not yet implemented");
    return rp->isAnyChildTaskSuccessful();
    /*PROTECTED REGION END*/
}
bool condition4368560569514553226(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1413201370590) ENABLED START*/
    AgentId agentID8 = 8;
    if (rp->getOwnID() == agentID8) {
        return alicaTests::TestWorldModel::getOne()->isTransitionCondition1413201370590();
    } else {
        return alicaTests::TestWorldModel::getTwo()->isTransitionCondition1413201370590();
    }
    /*PROTECTED REGION END*/
}
bool condition682216470625774387(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1414403842622) ENABLED START*/
    auto* worldModel = dynamic_cast<const alica_test::SchedWM*>(wm);
    return worldModel->planA2PlanB;
    /*PROTECTED REGION END*/
}
bool condition655002160731734731(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1418042683692) ENABLED START*/
    auto* worldModel = dynamic_cast<const alica_test::SchedWM*>(wm);
    return worldModel->planB2PlanA;
    /*PROTECTED REGION END*/
}
bool condition1237521027685048666(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1418825427317) ENABLED START*/
    LockedBlackboardRO bb(*(rp->getBasicPlan()->getBlackboard()));
    auto testWm = const_cast<alicaTests::TestWorldModel*>(dynamic_cast<const alicaTests::TestWorldModel*>(wm));
    testWm->passedParameters["planInputKey"] = bb.get<int>("planInputKey");
    return false;
    /*PROTECTED REGION END*/
}
bool condition1221637895518338620(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1418825428924) ENABLED START*/
    AgentId agentID8 = 8;
    if (rp->getOwnID() == agentID8) {
        return alicaTests::TestWorldModel::getOne()->isTransitionCondition1418825428924();
    } else {
        return alicaTests::TestWorldModel::getTwo()->isTransitionCondition1418825428924();
    }
    /*PROTECTED REGION END*/
}
bool condition3787001793582633602(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1522377944921) ENABLED START*/
    // static_assert(false, "Condition 1522377944921 with name MISSING_NAME is not yet implemented");
    LockedBlackboardRO bb(*input);
    return SimpleSwitches::isSet(bb.get<int>("idx"));
    /*PROTECTED REGION END*/
}
bool condition3726136276355540527(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1522377946607) ENABLED START*/
    return CounterClass::called == 1;
    /*PROTECTED REGION END*/
}
bool condition2452554857659522052(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1529456610697) ENABLED START*/
    auto* worldModel = dynamic_cast<const alica_test::SchedWM*>(wm);
    return worldModel->execBehaviourTest;
    /*PROTECTED REGION END*/
}
bool condition4108042962123065459(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1529456611916) ENABLED START*/
    // static_assert(false, "Condition 1529456611916 with name MISSING_NAME is not yet implemented");
    return false;
    /*PROTECTED REGION END*/
}
bool condition2619422076497988080(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1530004993493) ENABLED START*/
    auto* worldModel = dynamic_cast<const alica_test::SchedWM*>(wm);
    return worldModel->execOrderTest;
    /*PROTECTED REGION END*/
}
bool condition1648591654803570403(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1530004994611) ENABLED START*/
    AgentId agentID8 = 8;
    if (rp->getOwnID() == agentID8) {
        return alicaTests::TestWorldModel::getOne()->isTransitionCondition1413201389955() /*&& rp->allChildrenStatus(PlanStatus::Success)*/;
    } else {
        return alicaTests::TestWorldModel::getTwo()->isTransitionCondition1413201389955() /*&& rp->allChildrenStatus(PlanStatus::Success)*/;
    }
    /*PROTECTED REGION END*/
}
bool condition1311087067347475449(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1532424093178) ENABLED START*/
    // static_assert(false, "Condition 1532424093178 with name MISSING_NAME is not yet implemented");
    return CounterClass::called == 0;
    /*PROTECTED REGION END*/
}
bool condition1377356708472618789(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1532424113475) ENABLED START*/
    AgentId agentID8 = 8;
    if (rp->getOwnID() == agentID8) {
        return alicaTests::TestWorldModel::getOne()->isTransitionCondition1413201052549();
    } else {
        return alicaTests::TestWorldModel::getTwo()->isTransitionCondition1413201052549();
    }
    /*PROTECTED REGION END*/
}
bool condition2019050763618766552(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1588253347213) ENABLED START*/
    AgentId agentID8 = 8;
    if (rp->getOwnID() == agentID8) {
        return alicaTests::TestWorldModel::getOne()->isTransitionCondition1413201367990();
    } else {
        return alicaTests::TestWorldModel::getTwo()->isTransitionCondition1413201367990();
    }
    /*PROTECTED REGION END*/
}
bool condition3016035752801585170(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1588069612661) ENABLED START*/
    // static_assert(false, "Condition 1588069612661 with name 1588069612661 is not yet implemented");
    return SimpleSwitches::isSet(0);
    /*PROTECTED REGION END*/
}
bool condition1556522827919252115(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1588069615553) ENABLED START*/
    // static_assert(false, "Condition 1588069615553 with name 1588069615553 is not yet implemented");
    return SimpleSwitches::isSet(1);
    /*PROTECTED REGION END*/
}
bool condition4547372457936774346(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1588246141557) ENABLED START*/
    // static_assert(false, "Condition 1588246141557 with name 1588246141557 is not yet implemented");
    return alicaTests::TestWorldModel::getOne()->isPreCondition1840401110297459509();
    /*PROTECTED REGION END*/
}
bool condition3517323109117319233(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1588246144841) ENABLED START*/
    // static_assert(false, "Condition 1588246144841 with name 1588246144841 is not yet implemented");
    auto worldmodel = dynamic_cast<const alicaTests::TestWorldModel*>(wm);
    return worldmodel->isTransitionCondition1067314038887345208();
    /*PROTECTED REGION END*/
}
} /* namespace alica */
