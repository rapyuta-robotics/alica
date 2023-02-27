#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <memory>
using namespace std;

namespace alica
{
class AuthorityTestMaster1414403396328 : public DomainPlan
{
public:
    AuthorityTestMaster1414403396328(PlanContext& context);
    virtual ~AuthorityTestMaster1414403396328();
};

class UtilityFunction1414403396328 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1414403842622 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, AuthorityTestMaster1414403396328)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, AuthorityTestMaster1414403396328UtilityFunction)
} /* namespace alica */
