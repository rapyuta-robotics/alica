#include "Conditions.h"
#include <memory>

// TODO: simplify conditions
// TODO: replace world model bools
// TODO: remove duplicate conditions

namespace alica::tests
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

bool Entry2Wait(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = alica::LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1747408236004727286();
}
bool FailurePlan2FailureHandled(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = alica::LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    if (worldModel->transitionCondition3194919312481305139Enabled()) {
        return rp->isAnyChildStatus(PlanStatus::Failed);
    }
    return false;
}
bool TriggerCond(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return alica::LockedBlackboardRO(*input).get<bool>("trigger");
}
bool PlanB2PlanA(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::shared_ptr<alica_test::SchedWM> worldModel = alica::LockedBlackboardRO(*gb).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    return worldModel->planB2PlanA;
}
bool PlanA2PlanB(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::shared_ptr<alica_test::SchedWM> worldModel = alica::LockedBlackboardRO(*gb).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    return worldModel->planA2PlanB;
}
bool Default2EndTest(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return CounterClass::called == 4;
}
bool SecondTaskFirstState2SecondTaskSecondState(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = alica::LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1418825428924();
}
bool SecondCall2FirstCall(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = alica::LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    alica::LockedBlackboardRO bb(*(rp->getBasicPlan()->getBlackboard()));
    worldModel->passedParameters["planInputKey"] = bb.get<int64_t>("planInputKey");
    return false;
}
bool Init2Fail(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = alica::LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1446293122737278544();
}
bool Start2Init(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return CounterClass::called == 0;
}
bool StateOne2StateTwo(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = alica::LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1413201052549();
}
bool SwitchIsSet(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return SimpleSwitches::isSet(1);
}
bool Start2Finish(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = alica::LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1413201389955();
}
bool Fail2Failed(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = alica::LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1023566846009251524();
}
bool StateTwo2NewSuccessStateTwo(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = alica::LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1413201367990();
}
bool CounterClassCalled(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return false;
}
bool FirstTaskFirstState2FirstTaskSecondState(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = alica::LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1418825427317();
}
bool BehaviourSubPlan2ExecuteBehaviour(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::shared_ptr<alica_test::SchedWM> worldModel = alica::LockedBlackboardRO(*gb).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    return worldModel->transitionToExecuteBehaviour;
}
bool Init2Start(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = alica::LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1413201227586();
}
bool Start2ExecBehaviourTest(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::shared_ptr<alica_test::SchedWM> worldModel = alica::LockedBlackboardRO(*gb).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    return worldModel->execBehaviourTest;
}
bool Start2ExecOrderTest(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::shared_ptr<alica_test::SchedWM> worldModel = alica::LockedBlackboardRO(*gb).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    return worldModel->execOrderTest;
}
bool Init2End(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return CounterClass::called == 8;
}
bool CounterCalled(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    alica::LockedBlackboardRO bb(*input);
    return CounterClass::called == bb.get<int64_t>("numberOfCalls");
}
bool SwitchIsNotSet(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return SimpleSwitches::isSet(0);
}
bool Wait2Suc(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = alica::LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1067314038887345208();
}
bool TriggerFromInputCond(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    alica::LockedBlackboardRO bb(*input);
    return bb.get<bool>("result");
}
bool Decision2B(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::string value;
    rp->getParameter("TestValue", value);
    return value.compare("2") == 0;
}
bool Start2Default(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return CounterClass::called == 1;
}
bool SimpleSwitchIsSet(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    alica::LockedBlackboardRO bb(*input);
    return SimpleSwitches::isSet(bb.get<int64_t>("idx"));
}
bool BehaviourInSubPlan2EndTest(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return false;
}
bool Start2ExecOrderedSchedulingTest(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return false;
}
bool ExecBehaviour2SubPlan(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::shared_ptr<alica_test::SchedWM> worldModel = alica::LockedBlackboardRO(*gb).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    return worldModel->transitionToExecuteBehaviourInSubPlan;
}
bool Decision2A(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::string value;
    rp->getParameter("TestValue", value);
    return value.compare("1") == 0;
}
bool Other2NewSuccessStateOne(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = alica::LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1413201370590();
}
bool TestTracingMasterCondition(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = alica::LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isPreCondition1840401110297459509();
}

} /* namespace alica::tests */
