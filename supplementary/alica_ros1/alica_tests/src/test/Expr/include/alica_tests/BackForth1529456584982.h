#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class BackForth1529456584982 : public DomainPlan
{
public:
    BackForth1529456584982(PlanContext& context);
    virtual ~BackForth1529456584982();
};

class UtilityFunction1529456584982 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1529456610697 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1529456611916 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, BackForth1529456584982)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, BackForth1529456584982UtilityFunction)
} /* namespace alica */
