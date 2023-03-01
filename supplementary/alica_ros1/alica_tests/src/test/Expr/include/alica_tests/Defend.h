#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class Defend : public DomainPlan
{
public:
    Defend(PlanContext& context);
};

class PreCondition1402488993122 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1402488991641 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1402489065962 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1402489073613 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, Defend)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, DefendUtilityFunction)
} /* namespace alica */
