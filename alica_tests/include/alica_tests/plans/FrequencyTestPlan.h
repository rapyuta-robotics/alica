#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class FrequencyTestPlan : public BasicPlan
{
public:
    FrequencyTestPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, FrequencyTestPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, FrequencyTestPlanUtilityFunction)
} /* namespace alica */
