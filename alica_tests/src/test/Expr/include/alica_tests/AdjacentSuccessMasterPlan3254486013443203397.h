#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl3254486013443203397) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth3254486013443203397) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class AdjacentSuccessMasterPlan3254486013443203397 : public DomainPlan
{
public:
    AdjacentSuccessMasterPlan3254486013443203397(PlanContext& context);
    virtual ~AdjacentSuccessMasterPlan3254486013443203397();
    /*PROTECTED REGION ID(pub3254486013443203397) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro3254486013443203397) ENABLED START*/
    // Add/Override protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv3254486013443203397) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction3254486013443203397 : public BasicUtilityFunction
{
public:
    UtilityFunction3254486013443203397();
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
