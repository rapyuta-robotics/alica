#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class AuthorityTest1414403413451 : public DomainPlan
{
public:
    AuthorityTest1414403413451(PlanContext& context);
    virtual ~AuthorityTest1414403413451();
};

class UtilityFunction1414403413451 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, AuthorityTest1414403413451)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, AuthorityTest1414403413451UtilityFunction)
} /* namespace alica */
