#pragma once

#include <alica_tests/DomainCondition.h>

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class BehaviorSuccessSpamMaster : public BasicPlan
{
public:
    BehaviorSuccessSpamMaster(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, BehaviorSuccessSpamMaster)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, BehaviorSuccessSpamMasterUtilityFunction)
} /* namespace alica */
