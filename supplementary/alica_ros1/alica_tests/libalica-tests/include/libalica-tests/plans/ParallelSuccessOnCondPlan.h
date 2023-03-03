#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class ParallelSuccessOnCondPlan : public BasicPlan
{
public:
    ParallelSuccessOnCondPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, ParallelSuccessOnCondPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, ParallelSuccessOnCondPlanUtilityFunction)

} /* namespace alica */
