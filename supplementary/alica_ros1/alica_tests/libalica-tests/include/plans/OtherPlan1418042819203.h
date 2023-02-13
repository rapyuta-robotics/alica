#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class OtherPlan : public alica::BasicPlan
{
public:
    OtherPlan(alica::PlanContext& context);
    static std::unique_ptr<OtherPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::OtherPlan::create, OtherPlan)

class OtherPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    OtherPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<OtherPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::OtherPlanUtilityFunction::create, OtherPlanUtilityFunction)

} // namespace alica::tests
