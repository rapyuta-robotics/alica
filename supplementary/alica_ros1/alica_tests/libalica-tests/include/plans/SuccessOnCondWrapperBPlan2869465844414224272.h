#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class uccessOnCondWrapperBPlan : public alica::BasicPlan
{
public:
    uccessOnCondWrapperBPlan(alica::PlanContext& context);
    static std::unique_ptr<uccessOnCondWrapperBPlan> create(alica::PlanContext& context);
    virtual void onInit() override;
};
BOOST_DLL_ALIAS(alica::tests::uccessOnCondWrapperBPlan::create, uccessOnCondWrapperBPlan)

class uccessOnCondWrapperBPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    uccessOnCondWrapperBPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<uccessOnCondWrapperBPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::uccessOnCondWrapperBPlanUtilityFunction::create, uccessOnCondWrapperBPlanUtilityFunction)

} // namespace alica::tests
