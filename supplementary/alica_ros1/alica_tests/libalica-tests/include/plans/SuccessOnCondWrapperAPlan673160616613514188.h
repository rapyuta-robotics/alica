#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class SuccessOnCondWrapperAPlan : public alica::BasicPlan
{
public:
    SuccessOnCondWrapperAPlan(alica::PlanContext& context);
    static std::unique_ptr<SuccessOnCondWrapperAPlan> create(alica::PlanContext& context);
    virtual void onInit() override;
};
BOOST_DLL_ALIAS(alica::tests::SuccessOnCondWrapperAPlan::create, SuccessOnCondWrapperAPlan)

class SuccessOnCondWrapperAPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    SuccessOnCondWrapperAPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<SuccessOnCondWrapperAPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::SuccessOnCondWrapperAPlanUtilityFunction::create, SuccessOnCondWrapperAPlanUtilityFunction)

} // namespace alica::tests
