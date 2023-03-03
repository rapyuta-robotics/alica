#pragma once

#include <alica_tests/DomainCondition.h>

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class MasterSyncTransition : public BasicPlan
{
public:
    MasterSyncTransition(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, MasterSyncTransition)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, MasterSyncTransitionUtilityFunction)
} /* namespace alica */
