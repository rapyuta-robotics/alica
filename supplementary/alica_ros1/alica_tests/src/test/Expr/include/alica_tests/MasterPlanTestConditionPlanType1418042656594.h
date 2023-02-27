#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class MasterPlanTestConditionPlanType1418042656594 : public DomainPlan
{
public:
    MasterPlanTestConditionPlanType1418042656594(PlanContext& context);
    virtual ~MasterPlanTestConditionPlanType1418042656594();
};

class UtilityFunction1418042656594 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1418042683692 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, MasterPlanTestConditionPlanType1418042656594)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, MasterPlanTestConditionPlanType1418042656594UtilityFunction)
} /* namespace alica */
