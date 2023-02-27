#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class ConstraintTestPlan1414068524245 : public DomainPlan
{
public:
    ConstraintTestPlan1414068524245(PlanContext& context);
    virtual ~ConstraintTestPlan1414068524245();
};

class UtilityFunction1414068524245 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class RunTimeCondition1414068566297 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, ConstraintTestPlan1414068524245)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, ConstraintTestPlan1414068524245UtilityFunction)
} /* namespace alica */
