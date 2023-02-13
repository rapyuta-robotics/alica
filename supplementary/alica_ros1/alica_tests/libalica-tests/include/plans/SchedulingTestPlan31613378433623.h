#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class SchedulingTestPlan3 : public alica::BasicPlan
{
public:
    SchedulingTestPlan3(alica::PlanContext& context);
    static std::unique_ptr<SchedulingTestPlan3> create(alica::PlanContext& context);
    virtual void onInit();
    virtual void onTerminate();
};
BOOST_DLL_ALIAS(alica::tests::SchedulingTestPlan3::create, SchedulingTestPlan3)

class SchedulingTestPlan3UtilityFunction : public alica::BasicUtilityFunction
{
public:
    SchedulingTestPlan3UtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<SchedulingTestPlan3UtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::SchedulingTestPlan3UtilityFunction::create, SchedulingTestPlan3UtilityFunction)

} // namespace alica::tests
