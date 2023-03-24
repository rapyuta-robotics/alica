#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class MasterPlan : public BasicPlan
{
public:
    MasterPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, MasterPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, MasterPlanUtilityFunction)
} /* namespace alica */
