#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class MultiAgentTestPlan : public alica::BasicPlan
{
public:
    MultiAgentTestPlan(alica::PlanContext& context);
    static std::unique_ptr<MultiAgentTestPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::MultiAgentTestPlan::create, MultiAgentTestPlan)

class MultiAgentTestPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    MultiAgentTestPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<MultiAgentTestPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::MultiAgentTestPlanUtilityFunction::create, MultiAgentTestPlanUtilityFunction)

} // namespace alica::tests
