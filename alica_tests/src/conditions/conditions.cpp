#include <alica_tests/conditions/conditions.h>

#include <alica/test/CounterClass.h>
#include <alica_tests/SimpleSwitches.h>
#include <alica_tests/TestWorldModel.h>
#include <alica_tests/test_sched_world_model.h>
#include <engine/BasicPlan.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <iostream>
#include <optional>

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

namespace alica
{
bool IsAnyChildStatus(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    alica::LockedBlackboardRO bb(*input);
    return rp->isAnyChildStatus(bb.get<alica::PlanStatus>("childStatus"));
}

bool IsAnyChildStatusFailed(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return rp->isAnyChildStatus(alica::PlanStatus::Failed);
}

bool IsAnyChildTaskSuccessfull(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return rp->isAnyChildTaskSuccessful();
}

bool IsAnyChildStatusSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return rp->isAnyChildStatus(alica::PlanStatus::Success);
}

bool Entry2Wait(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1747408236004727286();
}
bool FailurePlan2FailureHandled(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    if (worldModel->transitionCondition3194919312481305139Enabled()) {
        return rp->isAnyChildStatus(PlanStatus::Failed);
    }
    return false;
}
bool TriggerCond(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    return LockedBlackboardRO(*input).get<bool>("trigger");
}
bool PlanB2PlanA(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    std::shared_ptr<alica_test::SchedWM> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    return worldModel->planB2PlanA;
}
bool PlanA2PlanB(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    std::shared_ptr<alica_test::SchedWM> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    return worldModel->planA2PlanB;
}
bool Default2EndTest(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    return CounterClass::called == 4;
}
bool SecondTaskFirstState2SecondTaskSecondState(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1418825428924();
}
bool SecondCall2FirstCall(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    LockedBlackboardRO bb(*(rp->getBasicPlan()->getBlackboard()));
    worldModel->passedParameters["planInputKey"] = bb.get<int64_t>("planInputKey");
    return false;
}
bool Init2Fail(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1446293122737278544();
}
bool Start2Init(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    return CounterClass::called == 0;
}
bool StateOne2StateTwo(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1413201052549();
}
bool SwitchIsSet(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    return SimpleSwitches::isSet(1);
}
bool Start2Finish(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1413201389955();
}
bool Fail2Failed(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1023566846009251524();
}
bool StateTwo2NewSuccessStateTwo(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1413201367990();
}
bool CounterClassCalled(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    return false;
}
bool FirstTaskFirstState2FirstTaskSecondState(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1418825427317();
}
bool BehaviourSubPlan2ExecuteBehaviour(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    std::shared_ptr<alica_test::SchedWM> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    return worldModel->transitionToExecuteBehaviour;
}
bool Init2Start(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1413201227586();
}
bool Start2ExecBehaviourTest(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    std::shared_ptr<alica_test::SchedWM> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    return worldModel->execBehaviourTest;
}
bool Start2ExecOrderTest(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    std::shared_ptr<alica_test::SchedWM> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    return worldModel->execOrderTest;
}
bool Init2End(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    return CounterClass::called == 8;
}
bool CounterCalled(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRO bb(*input);
    return CounterClass::called == bb.get<int64_t>("numberOfCalls");
}
bool SwitchIsNotSet(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    return SimpleSwitches::isSet(0);
}
bool Wait2Suc(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1067314038887345208();
}
bool TriggerFromInputCond(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRO bb(*input);
    return bb.get<bool>("result");
}
bool Start2Default(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    return CounterClass::called == 1;
}
bool SimpleSwitchIsSet(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRO bb(*input);
    return SimpleSwitches::isSet(bb.get<int64_t>("idx"));
}
bool BehaviourInSubPlan2EndTest(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    return false;
}
bool Start2ExecOrderedSchedulingTest(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    return false;
}
bool ExecBehaviour2SubPlan(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    std::shared_ptr<alica_test::SchedWM> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    return worldModel->transitionToExecuteBehaviourInSubPlan;
}
bool Other2NewSuccessStateOne(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isTransitionCondition1413201370590();
}
bool TestTracingMasterCondition(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isPreCondition1840401110297459509();
}
bool TestHasNoError(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRO globalBlackboard(*gb);
    return !globalBlackboard.get<std::optional<std::string>>("testError").has_value();
}
bool ExecOrderVectorSizeCheck(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRO globalBlackboard(*gb);
    LockedBlackboardRO localBlackboard(*input);
    if (!globalBlackboard.hasValue("execOrder")) {
        return false;
    }
    std::vector<std::string> execOrder = globalBlackboard.get<std::vector<std::string>>("execOrder");
    return execOrder.size() == localBlackboard.get<int64_t>("expected");
}
bool ContinueExecOrderTestCheck(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRO globalBlackboard(*gb);
    LockedBlackboardRO localBlackboard(*input);
    std::vector<std::string> execOrder = globalBlackboard.get<std::vector<std::string>>("execOrder");
    return execOrder.size() < localBlackboard.get<int64_t>("expected") && rp->amISuccessfulInAnyChild();
}
bool InitsNotFinishedCheck(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRO globalBlackboard(*gb);
    LockedBlackboardRO localBlackboard(*input);
    if (!globalBlackboard.hasValue("execOrder")) {
        return true;
    }
    std::vector<std::string> execOrder = globalBlackboard.get<std::vector<std::string>>("execOrder");
    return execOrder.size() < localBlackboard.get<int64_t>("expected");
}
} /* namespace alica */
