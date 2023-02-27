#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class ConstraintTestMaster1414068495566 : public DomainPlan
{
public:
    ConstraintTestMaster1414068495566(PlanContext& context);
    virtual ~ConstraintTestMaster1414068495566();
};

class UtilityFunction1414068495566 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, ConstraintTestMaster1414068495566)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, ConstraintTestMaster1414068495566UtilityFunction)
} /* namespace alica */
