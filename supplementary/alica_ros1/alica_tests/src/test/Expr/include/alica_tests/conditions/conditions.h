#pragma once

#include <boost/dll/alias.hpp>

namespace alica
{
class Blackboard;
class RunningPlan;

bool Entry2Wait(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool FailurePlan2FailureHandled(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool TriggerCond(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool PlanB2PlanA(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool PlanA2PlanB(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool Default2EndTest(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool SecondTaskFirstState2SecondTaskSecondState(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool SecondCall2FirstCall(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool Init2Fail(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool Start2Init(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool StateOne2StateTwo(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool SwitchIsSet(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool Start2Finish(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool Fail2Failed(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool StateTwo2NewSuccessStateTwo(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool CounterClassCalled(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool FirstTaskFirstState2FirstTaskSecondState(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool BehaviourSubPlan2ExecuteBehaviour(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool Init2Start(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool Start2ExecBehaviourTest(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool Start2ExecOrderTest(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool Init2End(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool CounterCalled(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool SwitchIsNotSet(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool Wait2Suc(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool TriggerFromInputCond(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool Decision2B(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool Start2Default(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool SimpleSwitchIsSet(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool BehaviourInSubPlan2EndTest(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool Start2ExecOrderedSchedulingTest(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool ExecBehaviour2SubPlan(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool Decision2A(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool Other2NewSuccessStateOne(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool TestTracingMasterCondition(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool IsAnyChildStatusSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);
bool IsAnyChildTaskSuccessfull(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);
bool IsAnyChildStatusFailed(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);
bool IsAnyChildStatus(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb);

BOOST_DLL_ALIAS(alica::Entry2Wait, Entry2Wait)
BOOST_DLL_ALIAS(alica::FailurePlan2FailureHandled, FailurePlan2FailureHandled)
BOOST_DLL_ALIAS(alica::TriggerCond, TriggerCond)
BOOST_DLL_ALIAS(alica::PlanB2PlanA, PlanB2PlanA)
BOOST_DLL_ALIAS(alica::PlanA2PlanB, PlanA2PlanB)
BOOST_DLL_ALIAS(alica::Default2EndTest, Default2EndTest)
BOOST_DLL_ALIAS(alica::SecondTaskFirstState2SecondTaskSecondState, SecondTaskFirstState2SecondTaskSecondState)
BOOST_DLL_ALIAS(alica::SecondCall2FirstCall, SecondCall2FirstCall)
BOOST_DLL_ALIAS(alica::Init2Fail, Init2Fail)
BOOST_DLL_ALIAS(alica::Start2Init, Start2Init)
BOOST_DLL_ALIAS(alica::StateOne2StateTwo, StateOne2StateTwo)
BOOST_DLL_ALIAS(alica::SwitchIsSet, SwitchIsSet)
BOOST_DLL_ALIAS(alica::Start2Finish, Start2Finish)
BOOST_DLL_ALIAS(alica::Fail2Failed, Fail2Failed)
BOOST_DLL_ALIAS(alica::StateTwo2NewSuccessStateTwo, StateTwo2NewSuccessStateTwo)
BOOST_DLL_ALIAS(alica::CounterClassCalled, CounterClassCalled)
BOOST_DLL_ALIAS(alica::FirstTaskFirstState2FirstTaskSecondState, FirstTaskFirstState2FirstTaskSecondState)
BOOST_DLL_ALIAS(alica::BehaviourSubPlan2ExecuteBehaviour, BehaviourSubPlan2ExecuteBehaviour)
BOOST_DLL_ALIAS(alica::Init2Start, Init2Start)
BOOST_DLL_ALIAS(alica::Start2ExecBehaviourTest, Start2ExecBehaviourTest)
BOOST_DLL_ALIAS(alica::Start2ExecOrderTest, Start2ExecOrderTest)
BOOST_DLL_ALIAS(alica::Init2End, Init2End)
BOOST_DLL_ALIAS(alica::CounterCalled, CounterCalled)
BOOST_DLL_ALIAS(alica::SwitchIsNotSet, SwitchIsNotSet)
BOOST_DLL_ALIAS(alica::Wait2Suc, Wait2Suc)
BOOST_DLL_ALIAS(alica::TriggerFromInputCond, TriggerFromInputCond)
BOOST_DLL_ALIAS(alica::Decision2B, Decision2B)
BOOST_DLL_ALIAS(alica::Start2Default, Start2Default)
BOOST_DLL_ALIAS(alica::SimpleSwitchIsSet, SimpleSwitchIsSet)
BOOST_DLL_ALIAS(alica::BehaviourInSubPlan2EndTest, BehaviourInSubPlan2EndTest)
BOOST_DLL_ALIAS(alica::Start2ExecOrderedSchedulingTest, Start2ExecOrderedSchedulingTest)
BOOST_DLL_ALIAS(alica::ExecBehaviour2SubPlan, ExecBehaviour2SubPlan)
BOOST_DLL_ALIAS(alica::Decision2A, Decision2A)
BOOST_DLL_ALIAS(alica::Other2NewSuccessStateOne, Other2NewSuccessStateOne)
BOOST_DLL_ALIAS(alica::TestTracingMasterCondition, TestTracingMasterCondition)
BOOST_DLL_ALIAS(alica::IsAnyChildStatusSuccess, IsAnyChildStatusSuccess)
BOOST_DLL_ALIAS(alica::IsAnyChildTaskSuccessfull, IsAnyChildTaskSuccessfull)
BOOST_DLL_ALIAS(alica::IsAnyChildStatusFailed, IsAnyChildStatusFailed)
BOOST_DLL_ALIAS(alica::IsAnyChildStatus, IsAnyChildStatus)
} /* namespace alica */
