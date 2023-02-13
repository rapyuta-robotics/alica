#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class SchedulingTestPlan2 : public alica::BasicPlan
{
public:
    SchedulingTestPlan2(alica::PlanContext& context);
    static std::unique_ptr<SchedulingTestPlan2> create(alica::PlanContext& context);
    virtual void onInit();
    virtual void onTerminate();
};
BOOST_DLL_ALIAS(alica::tests::SchedulingTestPlan2::create, SchedulingTestPlan2)

class SchedulingTestPlan2UtilityFunction : public alica::BasicUtilityFunction
{
public:
    SchedulingTestPlan2UtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<SchedulingTestPlan2UtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::SchedulingTestPlan2UtilityFunction::create, SchedulingTestPlan2UtilityFunction)

} // namespace alica::tests
