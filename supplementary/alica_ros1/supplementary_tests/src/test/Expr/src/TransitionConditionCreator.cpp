#include "supplementary_tests/TransitionConditionCreator.h"

#include "supplementary_tests/conditions/conditions.h"
#include <engine/IAlicaWorldModel.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>
#include <iostream>

namespace alica
{

TransitionConditionCreator::TransitionConditionCreator() {}

TransitionConditionCreator::~TransitionConditionCreator() {}

std::function<bool(const Blackboard*, const RunningPlan*, const IAlicaWorldModel*)> TransitionConditionCreator::createConditions(
        int64_t conditionId, TransitionConditionContext& context)
{
    switch (conditionId) {
    case 295816226925111421:
        return std::bind(conditionVariableHandlingStart295816226925111421, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    case 2011598442725310989:
        return std::bind(conditionDefaultCondition2011598442725310989, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    default:
        std::cerr << "TransitionConditionCreator: Unknown condition id requested: " << conditionId << std::endl;
        throw new std::exception();
        break;
    }
}
} /* namespace alica */
