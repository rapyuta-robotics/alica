#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class TestMasterPlan : public alica::BasicPlan
{
public:
    TestMasterPlan(alica::PlanContext& context);
    static std::unique_ptr<TestMasterPlan> create(alica::PlanContext& context);
    virtual void onInit() override;
};
BOOST_DLL_ALIAS(alica::tests::TestMasterPlan::create, TestMasterPlan)

class TestMasterPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    TestMasterPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<TestMasterPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::TestMasterPlanUtilityFunction::create, TestMasterPlanUtilityFunction)

} // namespace alica::tests
