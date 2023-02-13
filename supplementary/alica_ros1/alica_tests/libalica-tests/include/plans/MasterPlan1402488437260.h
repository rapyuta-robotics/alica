#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class MasterPlan : public alica::BasicPlan
{
public:
    MasterPlan(alica::PlanContext& context);
    static std::unique_ptr<MasterPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::MasterPlan::create, MasterPlan)

class MasterPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    MasterPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<MasterPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::MasterPlanUtilityFunction::create, MasterPlanUtilityFunction)

} // namespace alica::tests
