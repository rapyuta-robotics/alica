#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaWorldModel.h>
#include <engine/PlanAttachment.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1530069246103) ENABLED START*/
// Add inlcudes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1530069246103) ENABLED START*/
// Add other things here
/*PROTECTED REGION END*/
class FailsOnOne1530069246103 : public DomainPlan
{
public:
    FailsOnOne1530069246103(IAlicaWorldModel* wm);
    virtual ~FailsOnOne1530069246103();
    /*PROTECTED REGION ID(pub1530069246103) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    /*PROTECTED REGION ID(pro1530069246103) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1530069246103) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1530069246103 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class RunTimeCondition1530069251117 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
} /* namespace alica */
