#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class RuntimeConditionPlan1418042806575 : public DomainPlan
{
public:
    RuntimeConditionPlan1418042806575(PlanContext& context);
    virtual ~RuntimeConditionPlan1418042806575();
};

class UtilityFunction1418042806575 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class RunTimeCondition1418042967134 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, RuntimeConditionPlan1418042806575)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, RuntimeConditionPlan1418042806575UtilityFunction)
} /* namespace alica */
