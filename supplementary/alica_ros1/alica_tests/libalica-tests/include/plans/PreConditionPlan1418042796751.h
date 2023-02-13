#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class PreConditionPlan : public alica::BasicPlan
{
public:
    PreConditionPlan(alica::PlanContext& context);
    static std::unique_ptr<PreConditionPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::PreConditionPlan::create, PreConditionPlan)

class PreConditionPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    PreConditionPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<PreConditionPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::PreConditionPlanUtilityFunction::create, PreConditionPlanUtilityFunction)

} // namespace alica::tests
