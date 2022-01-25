#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaWorldModel.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1629895864090) ENABLED START*/
// Add additional includes here
namespace alica_test
{
class SchedWM;
}
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1629895864090) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class PlanAA1629895864090 : public DomainPlan
{
public:
    PlanAA1629895864090(IAlicaWorldModel* wm);
    virtual ~PlanAA1629895864090();
    /*PROTECTED REGION ID(pub1629895864090) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1629895864090) ENABLED START*/
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    virtual void onInit() override;
    virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1629895864090) ENABLED START*/
    // Add additional private methods here
    alica_test::SchedWM* _wm;
    /*PROTECTED REGION END*/
};

class UtilityFunction1629895864090 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
