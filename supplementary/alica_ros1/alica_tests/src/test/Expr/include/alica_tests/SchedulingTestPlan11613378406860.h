#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1613378406860) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1613378406860) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class SchedulingTestPlan11613378406860 : public DomainPlan
{
public:
    SchedulingTestPlan11613378406860(PlanContext& context);
    virtual ~SchedulingTestPlan11613378406860();
    /*PROTECTED REGION ID(pub1613378406860) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1613378406860) ENABLED START*/
    // Override these methods for your use case
    // virtual void run() override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    virtual void onInit();
    virtual void onTerminate();
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1613378406860) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1613378406860 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
