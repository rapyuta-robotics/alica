#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl2869465844414224272) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth2869465844414224272) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class SuccessOnCondWrapperB2869465844414224272 : public DomainPlan
{
public:
    SuccessOnCondWrapperB2869465844414224272(PlanContext& context);
    virtual ~SuccessOnCondWrapperB2869465844414224272();
    /*PROTECTED REGION ID(pub2869465844414224272) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro2869465844414224272) ENABLED START*/
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv2869465844414224272) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction2869465844414224272 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
