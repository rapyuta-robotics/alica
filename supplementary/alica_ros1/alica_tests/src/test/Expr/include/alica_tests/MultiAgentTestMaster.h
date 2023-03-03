#pragma once

#include <alica_tests/DomainCondition.h>

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class MultiAgentTestMaster : public BasicPlan
{
public:
    MultiAgentTestMaster(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, MultiAgentTestMaster)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, MultiAgentTestMasterUtilityFunction)
} /* namespace alica */
