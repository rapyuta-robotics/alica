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
bool GlobalCounterEqualTo(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRO globalBlackboard(*gb);
    LockedBlackboardRO localBlackboard(*input);
    return globalBlackboard.hasValue("counter") && globalBlackboard.get<int64_t>("counter") == localBlackboard.get<int64_t>("value");
}
bool GlobalCounterLessThan(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRO globalBlackboard(*gb);
    LockedBlackboardRO localBlackboard(*input);
    return globalBlackboard.hasValue("counter") && globalBlackboard.get<int64_t>("counter") < localBlackboard.get<int64_t>("value");
}
bool PlanAState2PlanBState(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRO globalBlackboard(*gb);
    std::string execOrder = globalBlackboard.get<std::string>("execOrder");
    std::string suffix = "BehAAA::Init\n";
    return globalBlackboard.get<int64_t>("counter") < 20 && execOrder.size() >= suffix.size() &&
           0 == execOrder.compare(execOrder.size() - suffix.size(), suffix.size(), suffix);
}
bool PlanBState2PlanAState(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRO globalBlackboard(*gb);
    std::string execOrder = globalBlackboard.get<std::string>("execOrder");
    std::string suffix = "BehBAA::Init\n";
    return globalBlackboard.get<int64_t>("counter") < 20 && execOrder.size() >= suffix.size() &&
           0 == execOrder.compare(execOrder.size() - suffix.size(), suffix.size(), suffix);
}
bool PlanARunCalled(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRO globalBlackboard(*gb);
    return globalBlackboard.hasValue("BehAAARunCount") && globalBlackboard.get<int64_t>("BehAAARunCount") > 0;
}
bool PlanBRunCalled(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRW globalBlackboard(*const_cast<Blackboard*>(gb));
    if (!globalBlackboard.hasValue("transitionCount")) {
        globalBlackboard.set("transitionCount", 0);
    }
    if (globalBlackboard.get<int64_t>("transitionCount") < 10 && globalBlackboard.hasValue("BehBAARunCount") &&
            globalBlackboard.get<int64_t>("BehBAARunCount") > 0) {
        globalBlackboard.set("transitionCount", globalBlackboard.get<int64_t>("transitionCount") + 1);
        return true;
    }
    return false;
}
bool OrderedRunFailureCond(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRO globalBlackboard(*gb);

    if (globalBlackboard.hasValue("transitionCount") && globalBlackboard.get<int64_t>("transitionCount") < 10) {
        // do more transitions before evaluating
        return false;
    }
    if (globalBlackboard.hasValue("BehAAARunOutOfOrder") && globalBlackboard.get<bool>("BehAAARunOutOfOrder")) {
        return true;
    }
    if (globalBlackboard.hasValue("PlanARunOutOfOrder") && globalBlackboard.get<bool>("PlanARunOutOfOrder")) {
        return true;
    }
    return !globalBlackboard.hasValue("BehAAARunCalled") || !globalBlackboard.get<bool>("BehAAARunCalled");
}
bool OrderedRunSuccessCond(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRO globalBlackboard(*gb);
    if (globalBlackboard.hasValue("transitionCount") && globalBlackboard.get<int64_t>("transitionCount") < 10) {
        // do more transitions before evaluating
        return false;
    }
    if (globalBlackboard.hasValue("BehAAARunOutOfOrder") && globalBlackboard.get<bool>("BehAAARunOutOfOrder")) {
        return false;
    }
    if (globalBlackboard.hasValue("PlanARunOutOfOrder") && globalBlackboard.get<bool>("PlanARunOutOfOrder")) {
        return false;
    }
    return globalBlackboard.hasValue("BehAAARunCalled") && globalBlackboard.get<bool>("BehAAARunCalled");
}
bool BehState2BehInSubPlanState(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRO globalBlackboard(*gb);
    if (globalBlackboard.hasValue("transitionCounter") && globalBlackboard.get<int64_t>("transitionCounter") >= 10) {
        return false;
    }
    std::string execOrder = globalBlackboard.get<std::string>("execOrder");
    std::string suffix = "TestBehaviour::Init\nTestBehaviour::Run\n";
    return execOrder.size() >= suffix.size() && 0 == execOrder.compare(execOrder.size() - suffix.size(), suffix.size(), suffix);
}
bool BehInSubPlanState2BehState(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRW globalBlackboard(*const_cast<Blackboard*>(gb));
    std::string execOrder = globalBlackboard.get<std::string>("execOrder");
    std::string suffix = "TestBehaviour::Term\nTestBehaviour::Init\nTestBehaviour::Run\n";
    if (execOrder.size() >= suffix.size() && 0 == execOrder.compare(execOrder.size() - suffix.size(), suffix.size(), suffix)) {
        globalBlackboard.set("transitionCounter", globalBlackboard.hasValue("transitionCounter") ? globalBlackboard.get<int64_t>("transitionCounter") + 1 : 1);
        return true;
    }
    return false;
}
bool BehaviourRunSchedulingCheck(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRO globalBlackboard(*gb);
    return globalBlackboard.get<int64_t>("BehAAARunCount") >= 10;
}
bool OrderedSchedulingCheck(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRO globalBlackboard(*gb);
    std::stringstream expectedOrder;
    for (int i = 0; i < 5; i++) {
        expectedOrder << "PlanA::Init\nPlanAA::Init\nBehAAA::Init\n"
                         "BehAAA::Term\nPlanAA::Term\nPlanA::Term\n"
                         "PlanB::Init\nPlanBA::Init\nBehBAA::Init\n"
                         "BehBAA::Term\nPlanBA::Term\nPlanB::Term\n";
    }
    std::string execOrder = globalBlackboard.get<std::string>("execOrder");
    return execOrder == expectedOrder.str();
}
bool ExecuteBehaviourTestSuccessCond(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    Logging::logInfo("execBehSucc");
    LockedBlackboardRW globalBlackboard(*const_cast<Blackboard*>(gb));
    if (globalBlackboard.hasValue("transitionCounter")) {
        Logging::logInfo("counter:") << globalBlackboard.get<int64_t>("transitionCounter");
    }

    return globalBlackboard.hasValue("transitionCounter") && globalBlackboard.get<int64_t>("transitionCounter") >= 10;
}
} /* namespace alica */
