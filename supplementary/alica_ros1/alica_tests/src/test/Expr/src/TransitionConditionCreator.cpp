#include "alica_tests/TransitionConditionCreator.h"

#include "alica_tests/conditions/conditions.h"
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>
#include <iostream>

namespace alica
{

TransitionConditionCreator::TransitionConditionCreator() {}

TransitionConditionCreator::~TransitionConditionCreator() {}

std::function<bool(const Blackboard*, const RunningPlan*, const Blackboard*)> TransitionConditionCreator::createConditions(
        int64_t conditionId, TransitionConditionContext& context)
{
    switch (conditionId) {
    case 1:
        return std::bind(conditionAnyChildSuccess1, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2:
        return std::bind(conditionAllChildSuccess2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 3:
        return std::bind(conditionAnyChildFailure3, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 4:
        return std::bind(conditionAllChildFailure4, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 19871606597697646:
        return std::bind(conditionEntry2Wait19871606597697646, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 190171326790683374:
        return std::bind(conditionFailurePlan2FailureHandled190171326790683374, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 330238006348384830:
        return std::bind(conditionisAnyChildTaskSuccessfull330238006348384830, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 593157092542472645:
        return std::bind(conditionTriggerCond593157092542472645, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 655002160731734731:
        return std::bind(conditionPlanB2PlanA655002160731734731, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 682216470625774387:
        return std::bind(conditionPlanA2PlanB682216470625774387, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 711536493236439192:
        return std::bind(conditionIsAnyChildStatusFailed711536493236439192, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 843443485857038179:
        return std::bind(conditionIsAnyChildStatus843443485857038179, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 1013158988206959873:
        return std::bind(conditionDefault2EndTest1013158988206959873, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 1221637895518338620:
        return std::bind(
                conditionSecondTaskFirstState2SecondTaskSecondState1221637895518338620, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 1237521027685048666:
        return std::bind(conditionSecondCall2FirstCall1237521027685048666, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 1291995818541962959:
        return std::bind(conditionInit2Fail1291995818541962959, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 1311087067347475449:
        return std::bind(conditionStart2Init1311087067347475449, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 1377356708472618789:
        return std::bind(conditionStateOne2StateTwo1377356708472618789, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 1556522827919252115:
        return std::bind(conditionSwitchIsSet1556522827919252115, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 1648591654803570403:
        return std::bind(conditionStart2Finish1648591654803570403, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 1678986049909129132:
        return std::bind(conditionDefaultCondition1678986049909129132, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 1770682125085719690:
        return std::bind(conditionFail2Failed1770682125085719690, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2019050763618766552:
        return std::bind(conditionStateTwo2NewSuccessStateTwo2019050763618766552, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2163654295690873706:
        return std::bind(conditionCounterClassCalled2163654295690873706, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2171152220550556375:
        return std::bind(
                conditionFirstTaskFirstState2FirstTaskSecondState2171152220550556375, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2205566100638019970:
        return std::bind(conditionBehaviourSubPlan2ExecuteBehaviour2205566100638019970, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2208457928613785430:
        return std::bind(conditionInit2Start2208457928613785430, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2452554857659522052:
        return std::bind(conditionStart2ExecBehaviourTest2452554857659522052, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2619422076497988080:
        return std::bind(conditionStart2ExecOrderTest2619422076497988080, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2711102114821139213:
        return std::bind(conditionInit2End2711102114821139213, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2872265442510628524:
        return std::bind(conditionAlwaysTrueCond2872265442510628524, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2901825906319407673:
        return std::bind(conditionCounterCalled2901825906319407673, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 3016035752801585170:
        return std::bind(conditionSwitchIsNotSet3016035752801585170, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 3517323109117319233:
        return std::bind(conditionWait2Suc3517323109117319233, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 3592699233854318376:
        return std::bind(conditionTriggerFromInputCond3592699233854318376, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 3684268241099966909:
        return std::bind(conditionDecision2B3684268241099966909, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 3726136276355540527:
        return std::bind(conditionStart2Default3726136276355540527, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 3787001793582633602:
        return std::bind(conditionSimpleSwitchIsSet3787001793582633602, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 3828316183435191952:
        return std::bind(conditionBehaviourInSubPlan2EndTest3828316183435191952, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 4108042962123065459:
        return std::bind(conditionStart2ExecOrderedSchedulingTest4108042962123065459, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 4244459279660861567:
        return std::bind(conditionExecBehaviour2SubPlan4244459279660861567, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 4281647834169813432:
        return std::bind(conditionDecision2A4281647834169813432, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 4368560569514553226:
        return std::bind(conditionOther2NewSuccessStateOne4368560569514553226, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 4547372457936774346:
        return std::bind(conditionTestTracingMasterCondition4547372457936774346, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    default:
        std::cerr << "TransitionConditionCreator: Unknown condition id requested: " << conditionId << std::endl;
        throw new std::exception();
        break;
    }
}
} /* namespace alica */
