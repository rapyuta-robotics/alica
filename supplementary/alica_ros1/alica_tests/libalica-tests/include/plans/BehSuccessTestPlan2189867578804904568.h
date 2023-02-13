#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class BehSuccessTestPlan : public alica::BasicPlan
{
public:
    BehSuccessTestPlan(alica::PlanContext& context);
    static std::unique_ptr<BehSuccessTestPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::BehSuccessTestPlan::create, BehSuccessTestPlan)

class BehSuccessTestPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    BehSuccessTestPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<BehSuccessTestPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::BehSuccessTestPlanUtilityFunction::create, BehSuccessTestPlanUtilityFunction)

} // namespace alica::tests
