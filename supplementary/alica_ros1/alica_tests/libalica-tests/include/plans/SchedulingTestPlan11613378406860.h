#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class SchedulingTestPlan1 : public alica::BasicPlan
{
public:
    SchedulingTestPlan1(alica::PlanContext& context);
    static std::unique_ptr<SchedulingTestPlan1> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::SchedulingTestPlan1::create, SchedulingTestPlan1)

class SchedulingTestPlan1UtilityFunction : public alica::BasicUtilityFunction
{
public:
    SchedulingTestPlan1UtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<SchedulingTestPlan1UtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::SchedulingTestPlan1UtilityFunction::create, SchedulingTestPlan1UtilityFunction)

} // namespace alica::tests
