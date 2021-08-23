#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1522377375148) ENABLED START*/
// Add inlcudes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1522377375148) ENABLED START*/
// Add other things here
/*PROTECTED REGION END*/
class BehaviorSuccessSpamMaster1522377375148 : public DomainPlan
{
public:
    BehaviorSuccessSpamMaster1522377375148();
    virtual ~BehaviorSuccessSpamMaster1522377375148();
    virtual void run(void* msg) override;
    /*PROTECTED REGION ID(pub1522377375148) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1522377375148) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1522377375148) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1522377375148 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1522377944921 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
class PreCondition1522377946607 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
} /* namespace alica */
