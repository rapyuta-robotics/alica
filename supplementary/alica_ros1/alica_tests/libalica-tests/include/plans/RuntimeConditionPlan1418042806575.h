#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class RuntimeConditionPlan : public alica::BasicPlan
{
public:
    RuntimeConditionPlan(alica::PlanContext& context);
    static std::unique_ptr<RuntimeConditionPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::RuntimeConditionPlan::create, RuntimeConditionPlan)

class RuntimeConditionPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    RuntimeConditionPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<RuntimeConditionPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::RuntimeConditionPlanUtilityFunction::create, RuntimeConditionPlanUtilityFunction)

} // namespace alica::tests
