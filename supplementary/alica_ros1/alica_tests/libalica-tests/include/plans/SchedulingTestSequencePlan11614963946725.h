#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class SchedulingTestSequencePlan1 : public alica::BasicPlan
{
public:
    SchedulingTestSequencePlan1(alica::PlanContext& context);
    static std::unique_ptr<SchedulingTestSequencePlan1> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::SchedulingTestSequencePlan1::create, SchedulingTestSequencePlan1)

class SchedulingTestSequencePlan1UtilityFunction : public alica::BasicUtilityFunction
{
public:
    SchedulingTestSequencePlan1UtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<SchedulingTestSequencePlan1UtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::SchedulingTestSequencePlan1UtilityFunction::create, SchedulingTestSequencePlan1UtilityFunction)

} // namespace alica::tests
