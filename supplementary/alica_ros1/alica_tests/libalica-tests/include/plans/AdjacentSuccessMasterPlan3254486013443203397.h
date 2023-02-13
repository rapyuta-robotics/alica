#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class AdjacentSuccessMasterPlan : public alica::BasicPlan
{
public:
    AdjacentSuccessMasterPlan(alica::PlanContext& context);
    static std::unique_ptr<AdjacentSuccessMasterPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::AdjacentSuccessMasterPlan::create, AdjacentSuccessMasterPlan)

class AdjacentSuccessMasterPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    AdjacentSuccessMasterPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<AdjacentSuccessMasterPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::AdjacentSuccessMasterPlanUtilityFunction::create, AdjacentSuccessMasterPlanUtilityFunction)

} // namespace alica::tests
