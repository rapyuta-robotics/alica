#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class TestInheritBlackboardMaster1179066429431332056 : public DomainPlan
{
public:
    TestInheritBlackboardMaster1179066429431332056(PlanContext& context);
    virtual ~TestInheritBlackboardMaster1179066429431332056();
};

class UtilityFunction1179066429431332056 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, TestInheritBlackboardMaster1179066429431332056)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestInheritBlackboardMaster1179066429431332056UtilityFunction)

} /* namespace alica */
