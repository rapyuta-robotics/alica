#include "alica/LegacyTransitionConditionCreator.h"

#include <alica/Master.h>
#include <alica/Move.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>
#include <iostream>

namespace alica
{

LegacyTransitionConditionCreator::LegacyTransitionConditionCreator() {}

LegacyTransitionConditionCreator::~LegacyTransitionConditionCreator() {}

std::function<bool(const Blackboard*, const RunningPlan*, const Blackboard*)> LegacyTransitionConditionCreator::createConditions(
        int64_t conditionId, TransitionConditionContext& context)
{
    int64_t preConditionId = context.preConditionId;
    switch (preConditionId) {
    default:
        std::cerr << "LegacyTransitionConditionCreator: Unknown condition id requested: " << preConditionId << std::endl;
        throw new std::exception();
        break;
    }
}
} /* namespace alica */
