#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class TestTracingSubPlan : public alica::BasicPlan
{
public:
    TestTracingSubPlan(alica::PlanContext& context);
    static std::unique_ptr<TestTracingSubPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::TestTracingSubPlan::create, TestTracingSubPlan)

class TestTracingSubPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    TestTracingSubPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<TestTracingSubPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::TestTracingSubPlanUtilityFunction::create, TestTracingSubPlanUtilityFunction)

} // namespace alica::tests
