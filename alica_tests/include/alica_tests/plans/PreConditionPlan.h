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
class PreConditionPlan : public BasicPlan
{
public:
    PreConditionPlan(PlanContext& context);
};

class PreConditionPlanUtilityFunction : public AlicaTestsUtilityFunction<PreConditionPlanUtilityFunction>
{
public:
    PreConditionPlanUtilityFunction(UtilityFunctionContext& context)
            : AlicaTestsUtilityFunction(context){};
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreConditionPlanPreCondition : public BasicCondition
{
public:
    PreConditionPlanPreCondition(ConditionContext& context)
            : BasicCondition(){};
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
    static std::unique_ptr<PreConditionPlanPreCondition> create(ConditionContext& context) { return std::make_unique<PreConditionPlanPreCondition>(context); };
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, PreConditionPlan)
BOOST_DLL_ALIAS(alica::PreConditionPlanUtilityFunction::create, PreConditionPlanUtilityFunction)
BOOST_DLL_ALIAS(alica::PreConditionPlanPreCondition::create, PreConditionPlanPreCondition)
} /* namespace alica */
