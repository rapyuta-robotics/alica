#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class MidFieldPlayPlan : public alica::BasicPlan
{
public:
    MidFieldPlayPlan(alica::PlanContext& context);
    static std::unique_ptr<MidFieldPlayPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::MidFieldPlayPlan::create, MidFieldPlayPlan)

class MidFieldPlayPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    MidFieldPlayPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<MidFieldPlayPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::MidFieldPlayPlanUtilityFunction::create, MidFieldPlayPlanUtilityFunction)

} // namespace alica::tests
