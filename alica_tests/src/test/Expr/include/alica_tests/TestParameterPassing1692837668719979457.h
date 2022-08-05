#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1692837668719979457) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1692837668719979457) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class TestParameterPassing1692837668719979457 : public DomainPlan
{
public:
    TestParameterPassing1692837668719979457(PlanContext& context);
    virtual ~TestParameterPassing1692837668719979457();
    /*PROTECTED REGION ID(pub1692837668719979457) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1692837668719979457) ENABLED START*/
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    virtual void onInit() override;
    // Add/Override protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1692837668719979457) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1692837668719979457 : public BasicUtilityFunction
{
public:
    UtilityFunction1692837668719979457();
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
