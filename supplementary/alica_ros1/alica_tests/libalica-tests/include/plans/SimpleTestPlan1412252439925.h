#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class SimpleTestPlan : public alica::BasicPlan
{
public:
    SimpleTestPlan(alica::PlanContext& context);
    static std::unique_ptr<SimpleTestPlan> create(alica::PlanContext& context);
    virtual void onInit() override;
};
BOOST_DLL_ALIAS(alica::tests::SimpleTestPlan::create, SimpleTestPlan)

class SimpleTestPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    SimpleTestPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<SimpleTestPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::SimpleTestPlanUtilityFunction::create, SimpleTestPlanUtilityFunction)

} // namespace alica::tests
