#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl3153116020668535682) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth3153116020668535682) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class SuccessOnCondPlan3153116020668535682 : public DomainPlan
{
public:
    SuccessOnCondPlan3153116020668535682(PlanContext& context);
    virtual ~SuccessOnCondPlan3153116020668535682();
    /*PROTECTED REGION ID(pub3153116020668535682) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro3153116020668535682) ENABLED START*/
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv3153116020668535682) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction3153116020668535682 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
