#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1629895837159) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1629895837159) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class PlanA1629895837159 : public DomainPlan
{
public:
    PlanA1629895837159();
    virtual ~PlanA1629895837159();
    virtual void run(void* msg) override;
    /*PROTECTED REGION ID(pub1629895837159) ENABLED START*/
    // Add additional public methods here
    virtual void onInit() override;
    virtual void onTerminate() override;
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1629895837159) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1629895837159) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1629895837159 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
