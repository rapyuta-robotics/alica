#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1418042819203) ENABLED START*/
// Add inlcudes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1418042819203) ENABLED START*/
// Add other things here
/*PROTECTED REGION END*/
class OtherPlan : public DomainPlan
{
public:
    OtherPlan();
    virtual ~OtherPlan();
    /*PROTECTED REGION ID(pub1418042819203) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1418042819203) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1418042819203) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1418042819203 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
