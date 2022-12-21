#include "supplementary_tests/LegacyTransitionConditionCreator.h"

#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>
#include <iostream>
#include <supplementary_tests/GSolver/GSolverMaster1417423751087.h>
#include <supplementary_tests/GSolver/GSolverTestPlan1417423757243.h>
#include <supplementary_tests/ProblemModule/ProbBuildingLevel11479557378264.h>
#include <supplementary_tests/ProblemModule/ProbBuildingLevel1_11479557664989.h>
#include <supplementary_tests/ProblemModule/ProblemBuildingMaster1479556022226.h>
#include <supplementary_tests/ProblemModule/QueryPlan11479556074049.h>
#include <supplementary_tests/ProblemModule/QueryPlan21479718449392.h>
#include <supplementary_tests/VariableHandling/Lvl11524452759599.h>
#include <supplementary_tests/VariableHandling/Lvl21524452793378.h>
#include <supplementary_tests/VariableHandling/Lvl31524452836022.h>
#include <supplementary_tests/VariableHandling/VHMaster1524452721452.h>

namespace alica
{

LegacyTransitionConditionCreator::LegacyTransitionConditionCreator() {}

LegacyTransitionConditionCreator::~LegacyTransitionConditionCreator() {}

std::function<bool(const Blackboard*, const RunningPlan*, const Blackboard*)> LegacyTransitionConditionCreator::createConditions(
        int64_t conditionId, TransitionConditionContext& context)
{
    int64_t preConditionId = context.preConditionId;
    switch (preConditionId) {
    case 1479557592662: {
        PreCondition1479557592662 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const Blackboard* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1524453491764: {
        PreCondition1524453491764 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const Blackboard* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    default:
        std::cerr << "LegacyTransitionConditionCreator: Unknown condition id requested: " << preConditionId << std::endl;
        throw new std::exception();
        break;
    }
}
} /* namespace alica */
