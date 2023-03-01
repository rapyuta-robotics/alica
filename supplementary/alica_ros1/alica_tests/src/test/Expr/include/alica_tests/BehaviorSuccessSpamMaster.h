#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class BehaviorSuccessSpamMaster : public DomainPlan
{
public:
    BehaviorSuccessSpamMaster(PlanContext& context);
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

BOOST_DLL_ALIAS(alica::BasicPlan::create, BehaviorSuccessSpamMaster)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, BehaviorSuccessSpamMasterUtilityFunction)
} /* namespace alica */
