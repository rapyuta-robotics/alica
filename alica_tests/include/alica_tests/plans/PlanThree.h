#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class PlanThree : public BasicPlan
{
public:
    PlanThree(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, PlanThree)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, PlanThreeUtilityFunction)
} /* namespace alica */
