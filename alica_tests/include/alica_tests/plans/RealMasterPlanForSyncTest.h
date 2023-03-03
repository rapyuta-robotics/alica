#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class RealMasterPlanForSyncTest : public BasicPlan
{
public:
    RealMasterPlanForSyncTest(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, RealMasterPlanForSyncTest)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, RealMasterPlanForSyncTestUtilityFunction)
} /* namespace alica */
