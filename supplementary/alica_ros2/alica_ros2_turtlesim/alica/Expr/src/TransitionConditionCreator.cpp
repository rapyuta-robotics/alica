#include "alica/TransitionConditionCreator.h"

#include "alica/conditions/conditions.h"
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>
#include <iostream>

namespace alica
{

TransitionConditionCreator::TransitionConditionCreator() {}

TransitionConditionCreator::~TransitionConditionCreator() {}

std::function<bool(const Blackboard*, const RunningPlan*, const Blackboard*)> TransitionConditionCreator::createConditions(
        TransitionConditionContext& context)
{
    int64_t conditionId = context.conditionConfId;
    switch (conditionId) {
    case 748720375848597116:
        return std::bind(conditionMove2Init748720375848597116, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 974606107671315045:
        return std::bind(conditionInit2Move974606107671315045, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2190266318562141841:
        return std::bind(conditionDefaultCondition2190266318562141841, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    default:
        std::cerr << "TransitionConditionCreator: Unknown condition id requested: " << conditionId << std::endl;
        throw new std::exception();
        break;
    }
}
} /* namespace alica */
