#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SuccessOnCondWrapperAPlan : public DomainPlan
{
public:
    SuccessOnCondWrapperAPlan(PlanContext& context);
};
class PreCondition4605367163774150375 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SuccessOnCondWrapperAPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SuccessOnCondWrapperAPlanUtilityFunction)

} /* namespace alica */
