#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class SchedulingTestSequenceSubPlan3 : public alica::BasicPlan
{
public:
    SchedulingTestSequenceSubPlan3(alica::PlanContext& context);
    static std::unique_ptr<SchedulingTestSequenceSubPlan3> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::SchedulingTestSequenceSubPlan3::create, SchedulingTestSequenceSubPlan3)

class SchedulingTestSequenceSubPlan3UtilityFunction : public alica::BasicUtilityFunction
{
public:
    SchedulingTestSequenceSubPlan3UtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<SchedulingTestSequenceSubPlan3UtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::SchedulingTestSequenceSubPlan3UtilityFunction::create, SchedulingTestSequenceSubPlan3UtilityFunction)

} // namespace alica::tests
