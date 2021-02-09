#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1412252439925) ENABLED START*/
// Add inlcudes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1412252439925) ENABLED START*/
// Add other things here
/*PROTECTED REGION END*/
class SimpleTestPlan1412252439925 : public DomainPlan
{
public:
    SimpleTestPlan1412252439925();
    virtual ~SimpleTestPlan1412252439925();
    /*PROTECTED REGION ID(pub1412252439925) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1412252439925) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1412252439925) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1412252439925 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1412781707952 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
class RunTimeCondition1412781693884 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
class PreCondition1412761926856 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
} /* namespace alica */
