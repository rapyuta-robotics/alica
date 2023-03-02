#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SuccessOnInitPlan : public DomainPlan
{
public:
    SuccessOnInitPlan(PlanContext& context);
};

class PreCondition4197030928062612573 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SuccessOnInitPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SuccessOnInitPlanUtilityFunction)
} /* namespace alica */
