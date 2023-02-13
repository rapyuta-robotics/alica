#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class MultiPlanInstanceSuccessTestPlan : public alica::BasicPlan
{
public:
    MultiPlanInstanceSuccessTestPlan(alica::PlanContext& context);
    static std::unique_ptr<MultiPlanInstanceSuccessTestPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::MultiPlanInstanceSuccessTestPlan::create, MultiPlanInstanceSuccessTestPlan)

class MultiPlanInstanceSuccessTestPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    MultiPlanInstanceSuccessTestPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<MultiPlanInstanceSuccessTestPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::MultiPlanInstanceSuccessTestPlanUtilityFunction::create, MultiPlanInstanceSuccessTestPlanUtilityFunction)

} // namespace alica::tests
