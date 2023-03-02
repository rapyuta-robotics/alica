#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class FrequencyTestPlan : public DomainPlan
{
public:
    FrequencyTestPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, FrequencyTestPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, FrequencyTestPlanUtilityFunction)
} /* namespace alica */
