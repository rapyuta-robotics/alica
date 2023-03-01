#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class ConstraintTestPlan : public DomainPlan
{
public:
    ConstraintTestPlan(PlanContext& context);
};

class RunTimeCondition1414068566297 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, ConstraintTestPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, ConstraintTestPlanUtilityFunction)
} /* namespace alica */
