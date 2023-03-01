#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class MultiAgentTestMaster : public DomainPlan
{
public:
    MultiAgentTestMaster(PlanContext& context);
};
class PreCondition1413201227586 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1413201389955 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, MultiAgentTestMaster)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, MultiAgentTestMasterUtilityFunction)
} /* namespace alica */
