#pragma once

#include <alica_tests/DomainCondition.h>

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SchedulingTestSequencePlan1 : public BasicPlan
{
public:
    SchedulingTestSequencePlan1(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SchedulingTestSequencePlan1)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SchedulingTestSequencePlan1UtilityFunction)

} /* namespace alica */
