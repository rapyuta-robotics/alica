#pragma once

#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <engine/BasicPlan.h>
#include <engine/BasicCondition.h>
#include <engine/BasicConstraint.h>
#include <boost/dll/alias.hpp>

namespace alica
{
class GSolverTestPlan : public BasicPlan
{
public:
    GSolverTestPlan(PlanContext& context);
    virtual ~GSolverTestPlan();
    static std::unique_ptr<GSolverTestPlan> create(alica::PlanContext& context);
};

BOOST_DLL_ALIAS(alica::GSolverTestPlan::create, GSolverTestPlan)

class GSolverTestPlanUtilityFunction : public BasicUtilityFunction
{
public:
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
    static std::unique_ptr<GSolverTestPlanUtilityFunction> create(alica::UtilityFunctionContext& context);
};

BOOST_DLL_ALIAS(alica::GSolverTestPlanUtilityFunction::create, GSolverTestPlanUtilityFunction)

class GSolverTestPlanRuntimeCondition : public BasicCondition
{
public:
    static std::unique_ptr<GSolverTestPlanRuntimeCondition> create(ConditionContext& context);
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
BOOST_DLL_ALIAS(alica::GSolverTestPlanRuntimeCondition::create, GSolverTestPlanRuntimeCondition)

class GSolverTestPlanRuntimeConditionConstraint : public BasicConstraint
{
public:
    void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
    static std::unique_ptr<GSolverTestPlanRuntimeConditionConstraint> create(alica::ConstraintContext&);
};
BOOST_DLL_ALIAS(alica::GSolverTestPlanRuntimeConditionConstraint::create, GSolverTestPlanRuntimeConditionConstraint)
} // namespace alica
