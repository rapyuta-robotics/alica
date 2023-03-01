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
class PreCondition1418042929966 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, PreConditionPlan)
BOOST_DLL_ALIAS(alica::PreConditionPlanUtilityFunction::create, PreConditionPlanUtilityFunction)
} /* namespace alica */
