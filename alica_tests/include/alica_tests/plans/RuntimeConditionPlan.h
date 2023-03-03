#pragma once

#include <engine/BasicCondition.h>

#include <alica_tests/util/AlicaTestsUtilityFunction.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class RuntimeConditionPlan : public BasicPlan
{
public:
    RuntimeConditionPlan(PlanContext& context);
};

class RuntimeConditionPlanUtilityFunction : public AlicaTestsUtilityFunction<RuntimeConditionPlanUtilityFunction>
{
public:
    RuntimeConditionPlanUtilityFunction(UtilityFunctionContext& context)
            : AlicaTestsUtilityFunction(context){};
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class RuntimeConditionPlanRuntimeCondition : public BasicCondition
{
public:
    RuntimeConditionPlanRuntimeCondition(ConditionContext& context)
            : BasicCondition(){};
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
    static std::unique_ptr<RuntimeConditionPlanRuntimeCondition> create(ConditionContext& context)
    {
        return std::make_unique<RuntimeConditionPlanRuntimeCondition>(context);
    };
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, RuntimeConditionPlan)
BOOST_DLL_ALIAS(alica::RuntimeConditionPlanUtilityFunction::create, RuntimeConditionPlanUtilityFunction)
BOOST_DLL_ALIAS(alica::RuntimeConditionPlanRuntimeCondition::create, RuntimeConditionPlanRuntimeCondition)
} /* namespace alica */
