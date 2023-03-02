#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class FailureHandlingMaster : public DomainPlan
{
public:
    FailureHandlingMaster(PlanContext& context);
};

class PreCondition488794245455049811 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, FailureHandlingMaster)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, FailureHandlingMasterUtilityFunction)
} /* namespace alica */
