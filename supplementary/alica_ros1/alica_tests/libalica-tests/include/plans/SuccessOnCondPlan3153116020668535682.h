#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class SuccessOnCondPlan : public alica::BasicPlan
{
public:
    SuccessOnCondPlan(alica::PlanContext& context);
    static std::unique_ptr<SuccessOnCondPlan> create(alica::PlanContext& context);
    virtual void onInit() override;
};
BOOST_DLL_ALIAS(alica::tests::SuccessOnCondPlan::create, SuccessOnCondPlan)

class SuccessOnCondPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    SuccessOnCondPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<SuccessOnCondPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::SuccessOnCondPlanUtilityFunction::create, SuccessOnCondPlanUtilityFunction)

} // namespace alica::tests
