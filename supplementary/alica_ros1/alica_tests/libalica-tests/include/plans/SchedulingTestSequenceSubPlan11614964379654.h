#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class SchedulingTestSequenceSubPlan1 : public alica::BasicPlan
{
public:
    SchedulingTestSequenceSubPlan1(alica::PlanContext& context);
    static std::unique_ptr<SchedulingTestSequenceSubPlan1> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::SchedulingTestSequenceSubPlan1::create, SchedulingTestSequenceSubPlan1)

class SchedulingTestSequenceSubPlan1UtilityFunction : public alica::BasicUtilityFunction
{
public:
    SchedulingTestSequenceSubPlan1UtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<SchedulingTestSequenceSubPlan1UtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::SchedulingTestSequenceSubPlan1UtilityFunction::create, SchedulingTestSequenceSubPlan1UtilityFunction)

} // namespace alica::tests
