#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicConstraint.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class ProblemDescriptor;
class AttackPlan : public DomainPlan
{
public:
    AttackPlan(PlanContext& context);
};

class Constraint1402489460549 : public BasicConstraint
{
    void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
};
class Constraint1402489462088 : public BasicConstraint
{
    void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, AttackPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, AttackPlanUtilityFunction)
} /* namespace alica */
