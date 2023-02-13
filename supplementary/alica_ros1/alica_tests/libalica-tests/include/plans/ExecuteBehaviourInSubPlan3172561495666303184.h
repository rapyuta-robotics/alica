#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class ExecuteBehaviourInSubPlan : public alica::BasicPlan
{
public:
    ExecuteBehaviourInSubPlan(alica::PlanContext& context);
    static std::unique_ptr<ExecuteBehaviourInSubPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::ExecuteBehaviourInSubPlan::create, ExecuteBehaviourInSubPlan)

class ExecuteBehaviourInSubPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    ExecuteBehaviourInSubPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<ExecuteBehaviourInSubPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::ExecuteBehaviourInSubPlanUtilityFunction::create, ExecuteBehaviourInSubPlanUtilityFunction)

} // namespace alica::tests
