#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1682631238618360548) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1682631238618360548) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class AdjacentSuccessSubPlan1682631238618360548 : public DomainPlan
{
public:
    AdjacentSuccessSubPlan1682631238618360548(PlanContext& context);
    virtual ~AdjacentSuccessSubPlan1682631238618360548();
    /*PROTECTED REGION ID(pub1682631238618360548) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1682631238618360548) ENABLED START*/
    // Add/Override protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1682631238618360548) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1682631238618360548 : public BasicUtilityFunction
{
public:
    UtilityFunction1682631238618360548(IAlicaLogger& logger);
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
