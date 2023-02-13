#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class FailurePlan : public alica::BasicPlan
{
public:
    FailurePlan(alica::PlanContext& context);
    static std::unique_ptr<FailurePlan> create(alica::PlanContext& context);
    virtual void onInit() override;
};
BOOST_DLL_ALIAS(alica::tests::FailurePlan::create, FailurePlan)

class FailurePlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    FailurePlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<FailurePlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::FailurePlanUtilityFunction::create, FailurePlanUtilityFunction)

} // namespace alica::tests
