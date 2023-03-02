#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <libalica-tests/util/AlicaTestsUtilityFunction.h>

namespace alica
{
class PreConditionPlan : public DomainPlan
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
class PreConditionPlanPreCondition : public DomainCondition
{
public:
    PreConditionPlanPreCondition(ConditionContext& context)
            : DomainCondition(){};
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
    static std::unique_ptr<PreConditionPlanPreCondition> create(ConditionContext& context) { return std::make_unique<PreConditionPlanPreCondition>(context); };
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, PreConditionPlan)
BOOST_DLL_ALIAS(alica::PreConditionPlanUtilityFunction::create, PreConditionPlanUtilityFunction)
BOOST_DLL_ALIAS(alica::PreConditionPlanPreCondition::create, PreConditionPlanPreCondition)
} /* namespace alica */
