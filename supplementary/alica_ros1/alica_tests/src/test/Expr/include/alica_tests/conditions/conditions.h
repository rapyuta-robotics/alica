#pragma once

/*PROTECTED REGION ID(conditionHeader) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
class Blackboard;
class RunningPlan;

bool conditionAnyChildSuccess1(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionAllChildSuccess2(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionAnyChildFailure3(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionAllChildFailure4(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionEntry2Wait19871606597697646(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionFailurePlan2FailureHandled190171326790683374(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionisAnyChildTaskSuccessfull330238006348384830(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionTriggerCond593157092542472645(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionPlanB2PlanA655002160731734731(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionPlanA2PlanB682216470625774387(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionIsAnyChildStatusFailed711536493236439192(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionIsAnyChildStatus843443485857038179(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionDefault2EndTest1013158988206959873(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionSecondTaskFirstState2SecondTaskSecondState1221637895518338620(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionSecondCall2FirstCall1237521027685048666(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionInit2Fail1291995818541962959(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionStart2Init1311087067347475449(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionStateOne2StateTwo1377356708472618789(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionSwitchIsSet1556522827919252115(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionStart2Finish1648591654803570403(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionDefaultCondition1678986049909129132(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionFail2Failed1770682125085719690(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionStateTwo2NewSuccessStateTwo2019050763618766552(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionCounterClassCalled2163654295690873706(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionFirstTaskFirstState2FirstTaskSecondState2171152220550556375(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionBehaviourSubPlan2ExecuteBehaviour2205566100638019970(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionInit2Start2208457928613785430(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionStart2ExecBehaviourTest2452554857659522052(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionStart2ExecOrderTest2619422076497988080(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionInit2End2711102114821139213(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionAlwaysTrueCond2872265442510628524(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionCounterCalled2901825906319407673(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionSwitchIsNotSet3016035752801585170(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionWait2Suc3517323109117319233(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionTriggerFromInputCond3592699233854318376(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionIsAnyChildStatusSuccess3604374027783683696(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionStart2Default3726136276355540527(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionSimpleSwitchIsSet3787001793582633602(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionBehaviourInSubPlan2EndTest3828316183435191952(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionStart2ExecOrderedSchedulingTest4108042962123065459(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionExecBehaviour2SubPlan4244459279660861567(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionOther2NewSuccessStateOne4368560569514553226(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
bool conditionTestTracingMasterCondition4547372457936774346(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb);
} /* namespace alica */
