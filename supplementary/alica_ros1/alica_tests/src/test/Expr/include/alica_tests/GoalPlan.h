#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class GoalPlan : public DomainPlan
{
public:
    GoalPlan(PlanContext& context);
};

class PreCondition1402489131988 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class RunTimeCondition1403773741874 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1402489174338 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1402489206278 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1402489218027 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PostCondition1402489620773 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, GoalPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, GoalPlanUtilityFunction)
} /* namespace alica */
