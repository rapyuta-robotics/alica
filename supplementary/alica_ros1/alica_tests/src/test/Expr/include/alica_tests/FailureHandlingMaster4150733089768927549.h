#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl4150733089768927549) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth4150733089768927549) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class FailureHandlingMaster4150733089768927549 : public DomainPlan
{
public:
    FailureHandlingMaster4150733089768927549(PlanContext& context);
    virtual ~FailureHandlingMaster4150733089768927549();
    /*PROTECTED REGION ID(pub4150733089768927549) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro4150733089768927549) ENABLED START*/
    // Override these methods for your use case
    // virtual void run() override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv4150733089768927549) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction4150733089768927549 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
