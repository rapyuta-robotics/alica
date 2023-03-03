#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SchedulingTestSequenceSubPlan1 : public BasicPlan
{
public:
    SchedulingTestSequenceSubPlan1(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SchedulingTestSequenceSubPlan1)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SchedulingTestSequenceSubPlan1UtilityFunction)

} /* namespace alica */
