#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl2189867578804904568) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth2189867578804904568) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class BehSuccessTestPlan2189867578804904568 : public DomainPlan
{
public:
    BehSuccessTestPlan2189867578804904568(PlanContext& context);
    virtual ~BehSuccessTestPlan2189867578804904568();
    /*PROTECTED REGION ID(pub2189867578804904568) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro2189867578804904568) ENABLED START*/
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv2189867578804904568) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction2189867578804904568 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
