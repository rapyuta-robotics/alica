#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class HandleFailExplicitMaster : public BasicPlan
{
public:
    HandleFailExplicitMaster(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, HandleFailExplicitMaster)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, HandleFailExplicitMasterUtilityFunction)
} /* namespace alica */
