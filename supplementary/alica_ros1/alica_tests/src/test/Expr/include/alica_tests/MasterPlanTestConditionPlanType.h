#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class MasterPlanTestConditionPlanType : public DomainPlan
{
public:
    MasterPlanTestConditionPlanType(PlanContext& context);
};
class PreCondition1418042683692 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, MasterPlanTestConditionPlanType)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, MasterPlanTestConditionPlanTypeUtilityFunction)
} /* namespace alica */
