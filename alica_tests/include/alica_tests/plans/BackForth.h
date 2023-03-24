#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class BackForth : public BasicPlan
{
public:
    BackForth(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, BackForth)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, BackForthUtilityFunction)
} /* namespace alica */
