#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class MultiAgentTestMaster1413200842973 : public DomainPlan
{
public:
    MultiAgentTestMaster1413200842973(PlanContext& context);
    virtual ~MultiAgentTestMaster1413200842973();
};

class UtilityFunction1413200842973 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
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

BOOST_DLL_ALIAS(alica::BasicPlan::create, MultiAgentTestMaster1413200842973)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, MultiAgentTestMaster1413200842973UtilityFunction)
} /* namespace alica */
