#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class BehaviorSuccessSpamMaster1522377375148 : public DomainPlan
{
public:
    BehaviorSuccessSpamMaster1522377375148(PlanContext& context);
    virtual ~BehaviorSuccessSpamMaster1522377375148();
};

class UtilityFunction1522377375148 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1522377944921 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1522377946607 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, BehaviorSuccessSpamMaster1522377375148)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, BehaviorSuccessSpamMaster1522377375148UtilityFunction)
} /* namespace alica */
