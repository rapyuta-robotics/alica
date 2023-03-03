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

class ConstraintTestPlan : public BasicPlan
{
public:
    ConstraintTestPlan(PlanContext& context);
};

class ConstraintTestPlanRuntimeCondition : public BasicCondition
{
public:
    ConstraintTestPlanRuntimeCondition(ConditionContext& context)
            : BasicCondition(){};
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
    static std::unique_ptr<ConstraintTestPlanRuntimeCondition> create(ConditionContext& context)
    {
        return std::make_unique<ConstraintTestPlanRuntimeCondition>(context);
    };
};

class ConstraintTestPlanRuntimeConditionConstraint : public BasicConstraint
{
public:
    ConstraintTestPlanRuntimeConditionConstraint(ConstraintContext& context)
            : BasicConstraint(){};
    void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
    static std::unique_ptr<ConstraintTestPlanRuntimeConditionConstraint> create(ConstraintContext& context)
    {
        return std::make_unique<ConstraintTestPlanRuntimeConditionConstraint>(context);
    };
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, ConstraintTestPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, ConstraintTestPlanUtilityFunction)
BOOST_DLL_ALIAS(alica::ConstraintTestPlanRuntimeCondition::create, ConstraintTestPlanRuntimeCondition)
BOOST_DLL_ALIAS(alica::ConstraintTestPlanRuntimeConditionConstraint::create, ConstraintTestPlanRuntimeConditionConstraint)
} /* namespace alica */
