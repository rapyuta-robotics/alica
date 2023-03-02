#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1626848999740) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1626848999740) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class FrequencyTestPlan1626848999740 : public DomainPlan
{
public:
    FrequencyTestPlan1626848999740(PlanContext& context);
    virtual ~FrequencyTestPlan1626848999740();
    /*PROTECTED REGION ID(pub1626848999740) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1626848999740) ENABLED START*/
    // Override these methods for your use case
    // virtual void run() override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1626848999740) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1626848999740 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
