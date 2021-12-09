#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaWorldModel.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1530004940652) ENABLED START*/
// Add inlcudes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1530004940652) ENABLED START*/
// Add other things here
/*PROTECTED REGION END*/
class HandleFailExplicitMaster1530004940652 : public DomainPlan
{
public:
    HandleFailExplicitMaster1530004940652(IAlicaWorldModel* wm);
    virtual ~HandleFailExplicitMaster1530004940652();
    /*PROTECTED REGION ID(pub1530004940652) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    /*PROTECTED REGION ID(pro1530004940652) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1530004940652) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1530004940652 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
