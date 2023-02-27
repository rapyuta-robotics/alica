#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class TestParameterPassing1692837668719979457 : public DomainPlan
{
public:
    TestParameterPassing1692837668719979457(PlanContext& context);
    virtual ~TestParameterPassing1692837668719979457();

protected:
    virtual void onInit() override;
};

class UtilityFunction1692837668719979457 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition2529456610600 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1529456610600 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, TestParameterPassing1692837668719979457)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestParameterPassing1692837668719979457UtilityFunction)

} /* namespace alica */
