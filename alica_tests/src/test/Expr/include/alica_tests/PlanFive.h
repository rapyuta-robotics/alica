#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class PlanFive : public BasicPlan
{
public:
    PlanFive(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, PlanFive)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, PlanFiveUtilityFunction)
} /* namespace alica */
