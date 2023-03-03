#pragma once

#include <engine/BasicCondition.h>

#include <boost/dll/alias.hpp>
#include <engine/BasicConstraint.h>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class ProblemDescriptor;

class GoalPlan : public BasicPlan
{
public:
    GoalPlan(PlanContext& context);
};

class GoalPlanPreCondition : public BasicCondition
{
public:
    GoalPlanPreCondition(ConditionContext& context)
            : BasicCondition(){};
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
    static std::unique_ptr<GoalPlanPreCondition> create(ConditionContext& context) { return std::make_unique<GoalPlanPreCondition>(context); };
};
class GoalPlanRuntimeCondition : public BasicCondition
{
public:
    GoalPlanRuntimeCondition(ConditionContext& context)
            : BasicCondition(){};
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
    static std::unique_ptr<GoalPlanRuntimeCondition> create(ConditionContext& context) { return std::make_unique<GoalPlanRuntimeCondition>(context); };
};
class GoalPlanPostCondition : public BasicCondition
{
public:
    GoalPlanPostCondition(ConditionContext& context)
            : BasicCondition(){};
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
    static std::unique_ptr<GoalPlanPostCondition> create(ConditionContext& context) { return std::make_unique<GoalPlanPostCondition>(context); };
};

class GoalPlanRuntimeConditionConstraint : public BasicConstraint
{
public:
    GoalPlanRuntimeConditionConstraint(ConstraintContext& context)
            : BasicConstraint(){};
    void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
    static std::unique_ptr<GoalPlanRuntimeConditionConstraint> create(ConstraintContext& context)
    {
        return std::make_unique<GoalPlanRuntimeConditionConstraint>(context);
    };
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, GoalPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, GoalPlanUtilityFunction)
BOOST_DLL_ALIAS(alica::GoalPlanPreCondition::create, GoalPlanPreCondition)
BOOST_DLL_ALIAS(alica::GoalPlanRuntimeCondition::create, GoalPlanRuntimeCondition)
BOOST_DLL_ALIAS(alica::GoalPlanPostCondition::create, GoalPlanPostCondition)
BOOST_DLL_ALIAS(alica::GoalPlanRuntimeConditionConstraint::create, GoalPlanRuntimeConditionConstraint)
} /* namespace alica */
