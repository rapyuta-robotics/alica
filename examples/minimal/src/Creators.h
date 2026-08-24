#pragma once

#include "CountUp.h"

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/IBehaviourCreator.h>
#include <engine/IConditionCreator.h>
#include <engine/IConstraintCreator.h>
#include <engine/IPlanCreator.h>
#include <engine/ITransitionConditionCreator.h>
#include <engine/IUtilityCreator.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <stdexcept>

namespace minimal
{

/**
 * The engine asks these factories to turn model IDs (from the .pml/.beh/.cnd
 * files) into live C++ objects. alica_dynamic_loading ships creators that
 * resolve them out of shared libraries via boost::dll; wiring them by hand
 * like this keeps the example free of Boost and makes the mapping explicit.
 */

class BehaviourCreator : public alica::IBehaviourCreator
{
public:
    std::unique_ptr<alica::BasicBehaviour> createBehaviour(int64_t behaviourId, alica::BehaviourContext& context) override
    {
        switch (behaviourId) {
        case 4001:
            return CountUp::create(context);
        default:
            throw std::runtime_error("Unknown behaviour id: " + std::to_string(behaviourId));
        }
    }
};

class PlanCreator : public alica::IPlanCreator
{
public:
    std::unique_ptr<alica::BasicPlan> createPlan(int64_t planId, alica::PlanContext& context) override
    {
        // A plain BasicPlan is enough unless you want onInit/run/onTerminate
        // hooks at plan level; subclass BasicPlan and return it here if you do.
        return alica::BasicPlan::create(context);
    }
};

class UtilityFunctionCreator : public alica::IUtilityCreator
{
public:
    std::shared_ptr<alica::BasicUtilityFunction> createUtility(int64_t utilityFunctionConfId, alica::UtilityFunctionContext& context) override
    {
        // The default utility function scores assignments purely by the
        // role/task priorities declared in Roleset.rst.
        return alica::BasicUtilityFunction::create(context);
    }
};

class TransitionConditionCreator : public alica::ITransitionConditionCreator
{
public:
    alica::TransitionConditionCallback createConditions(int64_t conditionId, alica::TransitionConditionContext& context) override
    {
        switch (conditionId) {
        case 3002:
            // CountReached: fires once the shared counter hits the target.
            return [](const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard) {
                alica::LockedBlackboardRO bb(*globalBlackboard);
                return bb.get<int64_t>(CountUp::COUNTER_KEY) >= CountUp::TARGET;
            };
        default:
            throw std::runtime_error("Unknown transition condition id: " + std::to_string(conditionId));
        }
    }
};

// This example declares no plan pre/runtime conditions and no constraints,
// so these two are never called. They still have to be supplied.
class ConditionCreator : public alica::IConditionCreator
{
public:
    std::shared_ptr<alica::BasicCondition> createConditions(int64_t conditionConfId, alica::ConditionContext& context) override { return nullptr; }
};

class ConstraintCreator : public alica::IConstraintCreator
{
public:
    std::shared_ptr<alica::BasicConstraint> createConstraint(int64_t constraintConfId, alica::ConstraintContext& context) override { return nullptr; }
};

} // namespace minimal
