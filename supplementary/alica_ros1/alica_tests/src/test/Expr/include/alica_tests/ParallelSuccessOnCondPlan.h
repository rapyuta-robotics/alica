#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class ParallelSuccessOnCondPlan : public DomainPlan
{
public:
    ParallelSuccessOnCondPlan(PlanContext& context);
};
class PreCondition2767999024419231358 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1470823850869867131 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, ParallelSuccessOnCondPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, ParallelSuccessOnCondPlanUtilityFunction)

} /* namespace alica */
