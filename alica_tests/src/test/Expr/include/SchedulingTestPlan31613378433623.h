#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaWorldModel.h>
#include <engine/PlanAttachment.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1613378433623) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1613378433623) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class SchedulingTestPlan31613378433623 : public DomainPlan
{
public:
    SchedulingTestPlan31613378433623(IAlicaWorldModel* wm);
    virtual ~SchedulingTestPlan31613378433623();
    /*PROTECTED REGION ID(pub1613378433623) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    /*PROTECTED REGION ID(pro1613378433623) ENABLED START*/
    // Add additional protected methods here
    virtual void onInit();
    virtual void onTerminate();
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1613378433623) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1613378433623 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */