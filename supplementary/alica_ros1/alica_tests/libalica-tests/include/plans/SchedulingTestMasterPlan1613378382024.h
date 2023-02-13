#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class SchedulingTestMasterPlan : public alica::BasicPlan
{
public:
    SchedulingTestMasterPlan(alica::PlanContext& context);
    static std::unique_ptr<SchedulingTestMasterPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::SchedulingTestMasterPlan::create, SchedulingTestMasterPlan)

class SchedulingTestMasterPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    SchedulingTestMasterPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<SchedulingTestMasterPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::SchedulingTestMasterPlanUtilityFunction::create, SchedulingTestMasterPlanUtilityFunction)

} // namespace alica::tests
