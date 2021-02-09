#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1402488634525) ENABLED START*/
// Add inlcudes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1402488634525) ENABLED START*/
// Add other things here
/*PROTECTED REGION END*/
class AttackPlan1402488634525 : public DomainPlan
{
public:
    AttackPlan1402488634525();
    virtual ~AttackPlan1402488634525();
    /*PROTECTED REGION ID(pub1402488634525) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1402488634525) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1402488634525) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1402488634525 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1402489460549 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
class PreCondition1402489462088 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
} /* namespace alica */
