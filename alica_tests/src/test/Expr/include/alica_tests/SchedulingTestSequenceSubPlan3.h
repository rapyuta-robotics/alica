#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SchedulingTestSequenceSubPlan3 : public BasicPlan
{
public:
    SchedulingTestSequenceSubPlan3(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SchedulingTestSequenceSubPlan3)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SchedulingTestSequenceSubPlan3UtilityFunction)

} /* namespace alica */
