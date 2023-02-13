#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class FrequencyTestPlan : public alica::BasicPlan
{
public:
    FrequencyTestPlan(alica::PlanContext& context);
    static std::unique_ptr<FrequencyTestPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::FrequencyTestPlan::create, FrequencyTestPlan)

class FrequencyTestPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    FrequencyTestPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<FrequencyTestPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::FrequencyTestPlanUtilityFunction::create, FrequencyTestPlanUtilityFunction)

} // namespace alica::tests
