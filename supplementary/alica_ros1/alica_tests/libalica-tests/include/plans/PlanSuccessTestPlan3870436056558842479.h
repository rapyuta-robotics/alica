#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class PlanSuccessTestPlan : public alica::BasicPlan
{
public:
    PlanSuccessTestPlan(alica::PlanContext& context);
    static std::unique_ptr<PlanSuccessTestPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::PlanSuccessTestPlan::create, PlanSuccessTestPlan)

class PlanSuccessTestPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    PlanSuccessTestPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<PlanSuccessTestPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::PlanSuccessTestPlanUtilityFunction::create, PlanSuccessTestPlanUtilityFunction)

} // namespace alica::tests
