#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class SchedulingTestSequenceSubPlan2 : public alica::BasicPlan
{
public:
    SchedulingTestSequenceSubPlan2(alica::PlanContext& context);
    static std::unique_ptr<SchedulingTestSequenceSubPlan2> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::SchedulingTestSequenceSubPlan2::create, SchedulingTestSequenceSubPlan2)

class SchedulingTestSequenceSubPlan2UtilityFunction : public alica::BasicUtilityFunction
{
public:
    SchedulingTestSequenceSubPlan2UtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<SchedulingTestSequenceSubPlan2UtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::SchedulingTestSequenceSubPlan2UtilityFunction::create, SchedulingTestSequenceSubPlan2UtilityFunction)

} // namespace alica::tests
