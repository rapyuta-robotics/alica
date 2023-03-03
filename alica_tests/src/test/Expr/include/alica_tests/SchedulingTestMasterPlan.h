#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SchedulingTestMasterPlan : public BasicPlan
{
public:
    SchedulingTestMasterPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SchedulingTestMasterPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SchedulingTestMasterPlanUtilityFunction)

} /* namespace alica */
