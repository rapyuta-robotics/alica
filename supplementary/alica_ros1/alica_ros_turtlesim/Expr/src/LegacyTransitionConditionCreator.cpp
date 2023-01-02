#include "alica_ros_turtlesim/LegacyTransitionConditionCreator.h"

#include <alica_ros_turtlesim/Master2425328142973735249.h>
#include <alica_ros_turtlesim/Move1889749086610694100.h>
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
    case 1136497454350831106: {
        PreCondition1136497454350831106 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const Blackboard* gb) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, gb);
        };
    }
    case 1597434482701133956: {
        PreCondition1597434482701133956 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const Blackboard* gb) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, gb);
        };
    }
    default:
        std::cerr << "LegacyTransitionConditionCreator: Unknown condition id requested: " << preConditionId << std::endl;
        throw new std::exception();
        break;
    }
}
} /* namespace alica */
