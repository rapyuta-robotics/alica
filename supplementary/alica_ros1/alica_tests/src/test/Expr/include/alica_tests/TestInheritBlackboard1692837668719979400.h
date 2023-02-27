#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class TestInheritBlackboard1692837668719979400 : public DomainPlan
{
public:
    TestInheritBlackboard1692837668719979400(PlanContext& context);
    virtual ~TestInheritBlackboard1692837668719979400();
};

class UtilityFunction1692837668719979400 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, TestInheritBlackboard1692837668719979400)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestInheritBlackboard1692837668719979400UtilityFunction)

} /* namespace alica */
