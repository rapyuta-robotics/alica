#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class TestTracingMasterPlan : public alica::BasicPlan
{
public:
    TestTracingMasterPlan(alica::PlanContext& context);
    static std::unique_ptr<TestTracingMasterPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::TestTracingMasterPlan::create, TestTracingMasterPlan)

class TestTracingMasterPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    TestTracingMasterPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<TestTracingMasterPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::TestTracingMasterPlanUtilityFunction::create, TestTracingMasterPlanUtilityFunction)

} // namespace alica::tests
