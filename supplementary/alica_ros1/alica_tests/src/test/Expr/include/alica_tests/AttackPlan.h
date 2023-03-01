#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class AttackPlan : public DomainPlan
{
public:
    AttackPlan(PlanContext& context);
};

class PreCondition1402489460549 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1402489462088 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, AttackPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, AttackPlanUtilityFunction)
} /* namespace alica */
