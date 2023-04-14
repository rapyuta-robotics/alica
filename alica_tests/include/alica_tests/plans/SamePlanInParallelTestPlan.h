#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SamePlanInParallelTestPlan : public BasicPlan
{
public:
    SamePlanInParallelTestPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SamePlanInParallelTestPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SamePlanInParallelTestPlanUtilityFunction)
} /* namespace alica */
