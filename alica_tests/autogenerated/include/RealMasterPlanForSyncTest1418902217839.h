#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1418902217839) ENABLED START*/
// Add inlcudes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1418902217839) ENABLED START*/
// Add other things here
/*PROTECTED REGION END*/
class RealMasterPlanForSyncTest : public DomainPlan
{
public:
    RealMasterPlanForSyncTest();
    virtual ~RealMasterPlanForSyncTest();
    /*PROTECTED REGION ID(pub1418902217839) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1418902217839) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1418902217839) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1418902217839 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
