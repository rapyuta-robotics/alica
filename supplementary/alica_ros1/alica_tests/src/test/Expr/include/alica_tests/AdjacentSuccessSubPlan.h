#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class AdjacentSuccessSubPlan : public DomainPlan
{
public:
    AdjacentSuccessSubPlan(PlanContext& context);
};

class PreCondition1067314038887345208 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition597347780541336226 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, AdjacentSuccessSubPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, AdjacentSuccessSubPlanUtilityFunction)
} /* namespace alica */
