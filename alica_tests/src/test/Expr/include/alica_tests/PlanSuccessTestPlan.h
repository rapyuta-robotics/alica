#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class PlanSuccessTestPlan : public DomainPlan
{
public:
    PlanSuccessTestPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, PlanSuccessTestPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, PlanSuccessTestPlanUtilityFunction)
} /* namespace alica */
