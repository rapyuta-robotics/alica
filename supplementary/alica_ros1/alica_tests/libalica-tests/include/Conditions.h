#pragma once

#include <boost/dll/alias.hpp>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

namespace utils
{
bool Entry2Wait(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool FailurePlan2FailureHandled(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool TriggerCond(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool PlanB2Plan(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool PlanA2Plan(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
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

BOOST_DLL_ALIAS(utils::Entry2Wait, Entry2Wait)
BOOST_DLL_ALIAS(utils::FailurePlan2FailureHandled, FailurePlan2FailureHandled)
BOOST_DLL_ALIAS(utils::TriggerCond, TriggerCond)
BOOST_DLL_ALIAS(utils::PlanB2Plan, PlanB2Plan)
BOOST_DLL_ALIAS(utils::PlanA2Plan, PlanA2Plan)
BOOST_DLL_ALIAS(utils::Default2EndTest, Default2EndTest)
BOOST_DLL_ALIAS(utils::SecondTaskFirstState2SecondTaskSecondState, SecondTaskFirstState2SecondTaskSecondState)
BOOST_DLL_ALIAS(utils::SecondCall2FirstCall, SecondCall2FirstCall)
BOOST_DLL_ALIAS(utils::Init2Fail, Init2Fail)
BOOST_DLL_ALIAS(utils::Start2Init, Start2Init)
BOOST_DLL_ALIAS(utils::StateOne2StateTwo, StateOne2StateTwo)
BOOST_DLL_ALIAS(utils::SwitchIsSet, SwitchIsSet)
BOOST_DLL_ALIAS(utils::Start2Finish, Start2Finish)
BOOST_DLL_ALIAS(utils::Fail2Failed, Fail2Failed)
BOOST_DLL_ALIAS(utils::StateTwo2NewSuccessStateTwo, StateTwo2NewSuccessStateTwo)
BOOST_DLL_ALIAS(utils::CounterClassCalled, CounterClassCalled)
BOOST_DLL_ALIAS(utils::FirstTaskFirstState2FirstTaskSecondState, FirstTaskFirstState2FirstTaskSecondState)
BOOST_DLL_ALIAS(utils::BehaviourSubPlan2ExecuteBehaviour, BehaviourSubPlan2ExecuteBehaviour)
BOOST_DLL_ALIAS(utils::Init2Start, Init2Start)
BOOST_DLL_ALIAS(utils::Start2ExecBehaviourTest, Start2ExecBehaviourTest)
BOOST_DLL_ALIAS(utils::Start2ExecOrderTest, Start2ExecOrderTest)
BOOST_DLL_ALIAS(utils::Init2End, Init2End)
BOOST_DLL_ALIAS(utils::CounterCalled, CounterCalled)
BOOST_DLL_ALIAS(utils::SwitchIsNotSet, SwitchIsNotSet)
BOOST_DLL_ALIAS(utils::Wait2Suc, Wait2Suc)
BOOST_DLL_ALIAS(utils::TriggerFromInputCond, TriggerFromInputCond)
BOOST_DLL_ALIAS(utils::Decision2B, Decision2B)
BOOST_DLL_ALIAS(utils::Start2Default, Start2Default)
BOOST_DLL_ALIAS(utils::SimpleSwitchIsSet, SimpleSwitchIsSet)
BOOST_DLL_ALIAS(utils::BehaviourInSubPlan2EndTest, BehaviourInSubPlan2EndTest)
BOOST_DLL_ALIAS(utils::Start2ExecOrderedSchedulingTest, Start2ExecOrderedSchedulingTest)
BOOST_DLL_ALIAS(utils::ExecBehaviour2SubPlan, ExecBehaviour2SubPlan)
BOOST_DLL_ALIAS(utils::Decision2A, Decision2A)
BOOST_DLL_ALIAS(utils::Other2NewSuccessStateOne, Other2NewSuccessStateOne)
BOOST_DLL_ALIAS(utils::TestTracingMasterCondition, TestTracingMasterCondition)
} /* namespace utils */
