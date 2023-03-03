#pragma once

#include <alica_tests/DomainCondition.h>

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <libalica-tests/util/AlicaTestsUtilityFunction.h>

namespace alica
{
class AuthorityTest : public BasicPlan
{
public:
    AuthorityTest(PlanContext& context);
};

class AuthorityTestUtilityFunction : public AlicaTestsUtilityFunction<AuthorityTestUtilityFunction>
{
public:
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
    AuthorityTestUtilityFunction(UtilityFunctionContext& context)
            : AlicaTestsUtilityFunction(context){};
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, AuthorityTest)
BOOST_DLL_ALIAS(alica::AuthorityTestUtilityFunction::create, AuthorityTestUtilityFunction)
} /* namespace alica */
