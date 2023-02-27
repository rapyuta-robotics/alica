#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class TestParameterPassingMaster1179066429431332055 : public DomainPlan
{
public:
    TestParameterPassingMaster1179066429431332055(PlanContext& context);
    virtual ~TestParameterPassingMaster1179066429431332055();

private:
    /*PROTECTED REGION ID(prv1179066429431332055) ENABLED START*/
    virtual void onInit() override;
};

class UtilityFunction1179066429431332055 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, TestParameterPassingMaster1179066429431332055)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestParameterPassingMaster1179066429431332055UtilityFunction)

} /* namespace alica */
