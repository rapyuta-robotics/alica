#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl3392981108193862307) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth3392981108193862307) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class MultiPlanInstanceSuccessTestPlan3392981108193862307 : public DomainPlan
{
public:
    MultiPlanInstanceSuccessTestPlan3392981108193862307(PlanContext& context);
    virtual ~MultiPlanInstanceSuccessTestPlan3392981108193862307();
    /*PROTECTED REGION ID(pub3392981108193862307) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro3392981108193862307) ENABLED START*/
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv3392981108193862307) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction3392981108193862307 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
