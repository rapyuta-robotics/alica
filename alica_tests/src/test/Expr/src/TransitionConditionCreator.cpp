#include "alica_tests/TransitionConditionCreator.h"

#include "alica_tests/conditions/conditions.h"
#include <iostream>

namespace alica
{

TransitionConditionCreator::TransitionConditionCreator() {}

TransitionConditionCreator::~TransitionConditionCreator() {}

// std::function<bool(const Blackboard*, const RunningPlan*, const IAlicaWorldModel*)> createConditions(int64_t conditionId)
std::function<bool(const Blackboard*, const RunningPlan*, const IAlicaWorldModel*)> TransitionConditionCreator::createConditions(int64_t conditionId)
{
    switch (conditionId) {
    case 2872265442510628524:
        return std::bind(condition2872265442510628524, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 3828316183435191952:
        return std::bind(condition3828316183435191952, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2205566100638019970:
        return std::bind(condition2205566100638019970, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2901825906319407673:
        return std::bind(condition2901825906319407673, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2163654295690873706:
        return std::bind(condition2163654295690873706, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 4281647834169813432:
        return std::bind(condition4281647834169813432, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 3684268241099966909:
        return std::bind(condition3684268241099966909, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 1013158988206959873:
        return std::bind(condition1013158988206959873, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 1678986049909129132:
        return std::bind(condition1678986049909129132, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 19871606597697646:
        return std::bind(condition19871606597697646, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 4244459279660861567:
        return std::bind(condition4244459279660861567, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 1770682125085719690:
        return std::bind(condition1770682125085719690, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 190171326790683374:
        return std::bind(condition190171326790683374, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2171152220550556375:
        return std::bind(condition2171152220550556375, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2711102114821139213:
        return std::bind(condition2711102114821139213, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 1291995818541962959:
        return std::bind(condition1291995818541962959, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2208457928613785430:
        return std::bind(condition2208457928613785430, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 843443485857038179:
        return std::bind(condition843443485857038179, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 711536493236439192:
        return std::bind(condition711536493236439192, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 3604374027783683696:
        return std::bind(condition3604374027783683696, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 330238006348384830:
        return std::bind(condition330238006348384830, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 4368560569514553226:
        return std::bind(condition4368560569514553226, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 682216470625774387:
        return std::bind(condition682216470625774387, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 655002160731734731:
        return std::bind(condition655002160731734731, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 1237521027685048666:
        return std::bind(condition1237521027685048666, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 1221637895518338620:
        return std::bind(condition1221637895518338620, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 3787001793582633602:
        return std::bind(condition3787001793582633602, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 3726136276355540527:
        return std::bind(condition3726136276355540527, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2452554857659522052:
        return std::bind(condition2452554857659522052, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 4108042962123065459:
        return std::bind(condition4108042962123065459, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2619422076497988080:
        return std::bind(condition2619422076497988080, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 1648591654803570403:
        return std::bind(condition1648591654803570403, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 1311087067347475449:
        return std::bind(condition1311087067347475449, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 1377356708472618789:
        return std::bind(condition1377356708472618789, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2019050763618766552:
        return std::bind(condition2019050763618766552, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 3016035752801585170:
        return std::bind(condition3016035752801585170, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 1556522827919252115:
        return std::bind(condition1556522827919252115, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 4547372457936774346:
        return std::bind(condition4547372457936774346, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 3517323109117319233:
        return std::bind(condition3517323109117319233, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    default:
        std::cerr << "TransitionConditionCreator: Unknown condition id requested: " << conditionId << std::endl;
        throw new std::exception();
        break;
    }
}
} /* namespace alica */
