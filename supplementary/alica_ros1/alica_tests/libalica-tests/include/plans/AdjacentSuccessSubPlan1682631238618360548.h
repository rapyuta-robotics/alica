#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class AdjacentSuccessSubPlan : public alica::BasicPlan
{
public:
    AdjacentSuccessSubPlan(alica::PlanContext& context);
    static std::unique_ptr<AdjacentSuccessSubPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::AdjacentSuccessSubPlan::create, AdjacentSuccessSubPlan)

class AdjacentSuccessSubPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    AdjacentSuccessSubPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<AdjacentSuccessSubPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::AdjacentSuccessSubPlanUtilityFunction::create, AdjacentSuccessSubPlanUtilityFunction)

} // namespace alica::tests
