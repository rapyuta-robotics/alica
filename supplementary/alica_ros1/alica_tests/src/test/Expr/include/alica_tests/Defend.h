#pragma once

#include <alica_tests/DomainCondition.h>

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class Defend : public BasicPlan
{
public:
    Defend(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, Defend)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, DefendUtilityFunction)
} /* namespace alica */
