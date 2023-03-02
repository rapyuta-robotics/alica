#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SuccessOnCondWrapperBPlan : public DomainPlan
{
public:
    SuccessOnCondWrapperBPlan(PlanContext& context);
};

class PreCondition914907830776317719 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SuccessOnCondWrapperBPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SuccessOnCondWrapperBPlanUtilityFunction)

} /* namespace alica */
