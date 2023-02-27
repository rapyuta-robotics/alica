#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class MultiAgentTestPlan1413200862180 : public DomainPlan
{
public:
    MultiAgentTestPlan1413200862180(PlanContext& context);
    virtual ~MultiAgentTestPlan1413200862180();
};

class UtilityFunction1413200862180 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1413201370590 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1413201052549 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1413201367990 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, MultiAgentTestPlan1413200862180)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, MultiAgentTestPlan1413200862180UtilityFunction)
} /* namespace alica */
