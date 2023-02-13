#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class GoalPlan : public alica::BasicPlan
{
public:
    GoalPlan(alica::PlanContext& context);
    static std::unique_ptr<GoalPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::GoalPlan::create, GoalPlan)

class GoalPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    GoalPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<GoalPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::GoalPlanUtilityFunction::create, GoalPlanUtilityFunction)

} // namespace alica::tests
