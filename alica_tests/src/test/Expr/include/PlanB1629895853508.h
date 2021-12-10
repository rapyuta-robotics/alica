#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaWorldModel.h>
#include <engine/PlanAttachment.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1629895853508) ENABLED START*/
// Add additional includes here
namespace alica_test
{
class SchedWM;
}
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1629895853508) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class PlanB1629895853508 : public DomainPlan
{
public:
    PlanB1629895853508(IAlicaWorldModel* wm);
    virtual ~PlanB1629895853508();
    /*PROTECTED REGION ID(pub1629895853508) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    /*PROTECTED REGION ID(pro1629895853508) ENABLED START*/
    // Add additional protected methods here
    virtual void onInit() override;
    virtual void onTerminate() override;
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1629895853508) ENABLED START*/
    // Add additional private methods here
    alica_test::SchedWM* _wm;
    /*PROTECTED REGION END*/
};

class UtilityFunction1629895853508 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
