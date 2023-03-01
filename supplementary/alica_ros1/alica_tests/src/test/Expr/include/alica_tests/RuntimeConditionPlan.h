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
class RuntimeConditionPlan : public DomainPlan
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
class RunTimeCondition1418042967134 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, RuntimeConditionPlan)
BOOST_DLL_ALIAS(alica::RuntimeConditionPlanUtilityFunction::create, RuntimeConditionPlanUtilityFunction)
} /* namespace alica */
