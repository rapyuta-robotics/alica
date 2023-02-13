#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class BehaviourTriggerTestPlan : public alica::BasicPlan
{
public:
    BehaviourTriggerTestPlan(alica::PlanContext& context);
    static std::unique_ptr<BehaviourTriggerTestPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::BehaviourTriggerTestPlan::create, BehaviourTriggerTestPlan)

class BehaviourTriggerTestPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    BehaviourTriggerTestPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<BehaviourTriggerTestPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::BehaviourTriggerTestPlanUtilityFunction::create, BehaviourTriggerTestPlanUtilityFunction)

} // namespace alica::tests
