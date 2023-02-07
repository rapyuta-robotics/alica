#include <alica_tests/conditions/conditions.h>

#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>
#include <iostream>

/*PROTECTED REGION ID(conditionSource) ENABLED START*/
#include <alica/test/CounterClass.h>
#include <alica_tests/SimpleSwitches.h>
#include <alica_tests/TestWorldModel.h>
#include <alica_tests/test_sched_world_model.h>
#include <engine/BasicPlan.h>

namespace alica
{

bool isSuccess(const alica::RunningPlan* rp)
{
    if (rp->isBehaviour()) {
        return rp->getStatus() == alica::PlanStatus::Success;
    } else {
        return rp->getActiveState()->isSuccessState();
    }
}

bool isFailure(const alica::RunningPlan* rp)
{
    if (rp->isBehaviour()) {
        return rp->getStatus() == alica::PlanStatus::Failed;
    } else {
        return rp->getActiveState()->isFailureState();
    }
}

} // namespace alica

/*PROTECTED REGION END*/

namespace alica
{
bool conditionAnyChildSuccess1(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition1) ENABLED START*/
    for (const alica::RunningPlan* child : rp->getChildren()) {
        if (isSuccess(child)) {
            return true;
        }
    }
    return false;
    /*PROTECTED REGION END*/
}
bool conditionAllChildSuccess2(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition2) ENABLED START*/
    for (const alica::RunningPlan* child : rp->getChildren()) {
        if (!isSuccess(child)) {
            return false;
        }
    }
    // In case of a state, make sure that all children are actually running
    if (rp->getActiveTriple().state) {
        return rp->getChildren().size() >= rp->getActiveTriple().state->getConfAbstractPlanWrappers().size();
    }
    return true;
    /*PROTECTED REGION END*/
}
bool conditionAnyChildFailure3(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition3) ENABLED START*/
    for (const alica::RunningPlan* child : rp->getChildren()) {
        if (isFailure(child)) {
            return true;
        }
    }
    return false;
    /*PROTECTED REGION END*/
}
bool conditionAllChildFailure4(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition4) ENABLED START*/
    for (const alica::RunningPlan* child : rp->getChildren()) {
        if (!isFailure(child)) {
            return false;
        }
    }
    // In case of a state, make sure that all children are actually running
    if (rp->getActiveTriple().state) {
        return rp->getChildren().size() >= rp->getActiveTriple().state->getConfAbstractPlanWrappers().size();
    }
    return true;
    /*PROTECTED REGION END*/
}
bool conditionEntry2Wait19871606597697646(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition19871606597697646) ENABLED START*/

    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1747408236004727286();
    /*PROTECTED REGION END*/
}
bool conditionFailurePlan2FailureHandled190171326790683374(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition190171326790683374) ENABLED START*/

    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    if (worldModel->transitionCondition3194919312481305139Enabled()) {
        return rp->isAnyChildStatus(PlanStatus::Failed);
    }
    return false;

    /*PROTECTED REGION END*/
}
bool conditionisAnyChildTaskSuccessfull330238006348384830(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition330238006348384830) ENABLED START*/
    return rp->isAnyChildTaskSuccessful();
    /*PROTECTED REGION END*/
}
bool conditionTriggerCond593157092542472645(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition593157092542472645) ENABLED START*/
    return LockedBlackboardRO(*input).get<bool>("trigger");
    /*PROTECTED REGION END*/
}
bool conditionPlanB2PlanA655002160731734731(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition655002160731734731) ENABLED START*/
    std::shared_ptr<alica_test::SchedWM> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");

    return worldModel->planB2PlanA;
    /*PROTECTED REGION END*/
}
bool conditionPlanA2PlanB682216470625774387(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition682216470625774387) ENABLED START*/
    std::shared_ptr<alica_test::SchedWM> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");

    return worldModel->planA2PlanB;
    /*PROTECTED REGION END*/
}
bool conditionIsAnyChildStatusFailed711536493236439192(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition711536493236439192) ENABLED START*/
    return rp->isAnyChildStatus(PlanStatus::Failed);
    /*PROTECTED REGION END*/
}
bool conditionIsAnyChildStatus843443485857038179(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition843443485857038179) ENABLED START*/
    LockedBlackboardRO bb(*input);
    return rp->isAnyChildStatus(bb.get<PlanStatus>("childStatus"));
    /*PROTECTED REGION END*/
}
bool conditionDefault2EndTest1013158988206959873(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition1013158988206959873) ENABLED START*/
    return CounterClass::called == 4;
    /*PROTECTED REGION END*/
}
bool conditionSecondTaskFirstState2SecondTaskSecondState1221637895518338620(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition1221637895518338620) ENABLED START*/
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1418825428924();
    /*PROTECTED REGION END*/
}
bool conditionSecondCall2FirstCall1237521027685048666(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition1237521027685048666) ENABLED START*/
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    LockedBlackboardRO bb(*(rp->getBasicPlan()->getBlackboard()));
    worldModel->passedParameters["planInputKey"] = bb.get<int64_t>("planInputKey");
    return false;
    /*PROTECTED REGION END*/
}
bool conditionInit2Fail1291995818541962959(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition1291995818541962959) ENABLED START*/
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1446293122737278544();
    /*PROTECTED REGION END*/
}
bool conditionStart2Init1311087067347475449(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition1311087067347475449) ENABLED START*/
    return CounterClass::called == 0;
    /*PROTECTED REGION END*/
}
bool conditionStateOne2StateTwo1377356708472618789(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition1377356708472618789) ENABLED START*/
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1413201052549();
    /*PROTECTED REGION END*/
}
bool conditionSwitchIsSet1556522827919252115(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition1556522827919252115) ENABLED START*/
    return SimpleSwitches::isSet(1);
    /*PROTECTED REGION END*/
}
bool conditionStart2Finish1648591654803570403(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition1648591654803570403) ENABLED START*/

    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1413201389955();
    /*PROTECTED REGION END*/
}
bool conditionDefaultCondition1678986049909129132(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition1678986049909129132) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
bool conditionFail2Failed1770682125085719690(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition1770682125085719690) ENABLED START*/
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");

    return worldModel->isTransitionCondition1023566846009251524();
    /*PROTECTED REGION END*/
}
bool conditionStateTwo2NewSuccessStateTwo2019050763618766552(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition2019050763618766552) ENABLED START*/
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1413201367990();
    /*PROTECTED REGION END*/
}
bool conditionCounterClassCalled2163654295690873706(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition2163654295690873706) ENABLED START*/
    LockedBlackboardRO bb(*input);
    return CounterClass::called == bb.get<int64_t>("numberOfCalls");
    /*PROTECTED REGION END*/
}
bool conditionFirstTaskFirstState2FirstTaskSecondState2171152220550556375(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition2171152220550556375) ENABLED START*/
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1418825427317();
    /*PROTECTED REGION END*/
}
bool conditionBehaviourSubPlan2ExecuteBehaviour2205566100638019970(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition2205566100638019970) ENABLED START*/
    std::shared_ptr<alica_test::SchedWM> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    return worldModel->transitionToExecuteBehaviour;
    /*PROTECTED REGION END*/
}
bool conditionInit2Start2208457928613785430(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition2208457928613785430) ENABLED START*/
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1413201227586();
    /*PROTECTED REGION END*/
}
bool conditionStart2ExecBehaviourTest2452554857659522052(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition2452554857659522052) ENABLED START*/
    std::shared_ptr<alica_test::SchedWM> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    return worldModel->execBehaviourTest;
    /*PROTECTED REGION END*/
}
bool conditionStart2ExecOrderTest2619422076497988080(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition2619422076497988080) ENABLED START*/
    std::shared_ptr<alica_test::SchedWM> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    return worldModel->execOrderTest;
    /*PROTECTED REGION END*/
}
bool conditionInit2End2711102114821139213(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition2711102114821139213) ENABLED START*/
    return CounterClass::called == 8;
    /*PROTECTED REGION END*/
}
bool conditionAlwaysTrueCond2872265442510628524(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition2872265442510628524) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
bool conditionCounterCalled2901825906319407673(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition2901825906319407673) ENABLED START*/
    LockedBlackboardRO bb(*input);
    return CounterClass::called == bb.get<int64_t>("numberOfCalls");
    /*PROTECTED REGION END*/
}
bool conditionSwitchIsNotSet3016035752801585170(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition3016035752801585170) ENABLED START*/
    return SimpleSwitches::isSet(0);
    /*PROTECTED REGION END*/
}
bool conditionWait2Suc3517323109117319233(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition3517323109117319233) ENABLED START*/

    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1067314038887345208();
    /*PROTECTED REGION END*/
}
bool conditionTriggerFromInputCond3592699233854318376(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition3592699233854318376) ENABLED START*/
    LockedBlackboardRO bb(*input);
    return bb.get<bool>("result");
    /*PROTECTED REGION END*/
}
bool conditionIsAnyChildStatusSuccess3604374027783683696(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition3604374027783683696) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
bool conditionDecision2B3684268241099966909(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition3684268241099966909) ENABLED START*/
    std::string value;
    rp->getParameter("TestValue", value);
    return value.compare("2") == 0;
    /*PROTECTED REGION END*/
}
bool conditionStart2Default3726136276355540527(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition3726136276355540527) ENABLED START*/
    return CounterClass::called == 1;
    /*PROTECTED REGION END*/
}
bool conditionSimpleSwitchIsSet3787001793582633602(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition3787001793582633602) ENABLED START*/
    LockedBlackboardRO bb(*input);
    return SimpleSwitches::isSet(bb.get<int64_t>("idx"));
    /*PROTECTED REGION END*/
}
bool conditionBehaviourInSubPlan2EndTest3828316183435191952(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition3828316183435191952) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
bool conditionStart2ExecOrderedSchedulingTest4108042962123065459(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition4108042962123065459) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
bool conditionExecBehaviour2SubPlan4244459279660861567(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition4244459279660861567) ENABLED START*/
    std::shared_ptr<alica_test::SchedWM> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    return worldModel->transitionToExecuteBehaviourInSubPlan;
    /*PROTECTED REGION END*/
}
bool conditionDecision2A4281647834169813432(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition4281647834169813432) ENABLED START*/
    std::string value;
    rp->getParameter("TestValue", value);
    return value.compare("1") == 0;
    /*PROTECTED REGION END*/
}
bool conditionOther2NewSuccessStateOne4368560569514553226(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition4368560569514553226) ENABLED START*/
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1413201370590();
    /*PROTECTED REGION END*/
}
bool conditionTestTracingMasterCondition4547372457936774346(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(condition4547372457936774346) ENABLED START*/
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isPreCondition1840401110297459509();
    /*PROTECTED REGION END*/
}
} /* namespace alica */
