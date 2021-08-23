#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1614964478264) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1614964478264) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class SchedulingTestSequenceSubPlan31614964478264 : public DomainPlan
{
public:
    SchedulingTestSequenceSubPlan31614964478264();
    virtual ~SchedulingTestSequenceSubPlan31614964478264();
    virtual void run(void* msg) override;
    /*PROTECTED REGION ID(pub1614964478264) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1614964478264) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1614964478264) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1614964478264 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
