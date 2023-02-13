#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class OrderedSchedulingTestPlan : public alica::BasicPlan
{
public:
    OrderedSchedulingTestPlan(alica::PlanContext& context);
    static std::unique_ptr<OrderedSchedulingTestPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::OrderedSchedulingTestPlan::create, OrderedSchedulingTestPlan)

class OrderedSchedulingTestPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    OrderedSchedulingTestPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<OrderedSchedulingTestPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::OrderedSchedulingTestPlanUtilityFunction::create, OrderedSchedulingTestPlanUtilityFunction)

} // namespace alica::tests
