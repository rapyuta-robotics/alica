#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class PlanOne : public BasicPlan
{
public:
    PlanOne(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, PlanOne)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, PlanOneUtilityFunction)
} /* namespace alica */
