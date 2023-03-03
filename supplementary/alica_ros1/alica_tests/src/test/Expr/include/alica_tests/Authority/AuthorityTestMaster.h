#pragma once

#include <alica_tests/DomainCondition.h>

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <memory>
using namespace std;

namespace alica
{
class AuthorityTestMaster : public BasicPlan
{
public:
    AuthorityTestMaster(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, AuthorityTestMaster)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, AuthorityTestMasterUtilityFunction)
} /* namespace alica */
