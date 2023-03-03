#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class ReadConfInPlantype : public BasicPlan
{
public:
    ReadConfInPlantype(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, ReadConfInPlantype)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, ReadConfInPlantypeUtilityFunction)
} /* namespace alica */
