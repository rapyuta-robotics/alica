#pragma once

/*PROTECTED REGION ID(conditionHeader) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
class Blackboard;
class RunningPlan;

bool conditionEntry2Wait19871606597697646(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionFailurePlan2FailureHandled190171326790683374(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionisAnyChildTaskSuccessfull330238006348384830(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionPlanB2PlanA655002160731734731(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionPlanA2PlanB682216470625774387(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionIsAnyChildStatusFailed711536493236439192(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionIsAnyChildStatus843443485857038179(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionDefault2EndTest1013158988206959873(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionSecondTaskFirstState2SecondTaskSecondState1221637895518338620(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionSecondCall2FirstCall1237521027685048666(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionInit2Fail1291995818541962959(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionStart2Init1311087067347475449(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionStateOne2StateTwo1377356708472618789(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionSwitchIsSet1556522827919252115(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionStart2Finish1648591654803570403(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionDefaultCondition1678986049909129132(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionFail2Failed1770682125085719690(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionStateTwo2NewSuccessStateTwo2019050763618766552(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionCounterClassCalled2163654295690873706(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionFirstTaskFirstState2FirstTaskSecondState2171152220550556375(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionBehaviourSubPlan2ExecuteBehaviour2205566100638019970(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionInit2Start2208457928613785430(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionStart2ExecBehaviourTest2452554857659522052(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionStart2ExecOrderTest2619422076497988080(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionInit2End2711102114821139213(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionAlwaysTrue2872265442510628524(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionCounterCalled2901825906319407673(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionSwitchIsNotSet3016035752801585170(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionWait2Suc3517323109117319233(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionIsAnyChildStatusSuccess3604374027783683696(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionDecision2B3684268241099966909(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionStart2Default3726136276355540527(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionSimpleSwitchIsSet3787001793582633602(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionBehaviourInSubPlan2EndTest3828316183435191952(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionStart2ExecOrderedSchedulingTest4108042962123065459(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionExecBehaviour2SubPlan4244459279660861567(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionDecision2A4281647834169813432(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionOther2NewSuccessStateOne4368560569514553226(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
bool conditionTestTracingMasterCondition4547372457936774346(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels);
} /* namespace alica */
