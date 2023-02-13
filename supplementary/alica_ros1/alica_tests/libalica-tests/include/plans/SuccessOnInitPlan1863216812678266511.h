#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class SuccessOnInitPlan : public alica::BasicPlan
{
public:
    SuccessOnInitPlan(alica::PlanContext& context);
    static std::unique_ptr<SuccessOnInitPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::SuccessOnInitPlan::create, SuccessOnInitPlan)

class SuccessOnInitPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    SuccessOnInitPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<SuccessOnInitPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::SuccessOnInitPlanUtilityFunction::create, SuccessOnInitPlanUtilityFunction)

} // namespace alica::tests
