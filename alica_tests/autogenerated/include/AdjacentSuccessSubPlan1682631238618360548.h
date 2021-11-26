#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaWorldModel.h>
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
    AdjacentSuccessSubPlan1682631238618360548(IAlicaWorldModel* wm);
    virtual ~AdjacentSuccessSubPlan1682631238618360548();
    /*PROTECTED REGION ID(pub1682631238618360548) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
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
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition3875618235052823378 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
class PreCondition3441061963559991094 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
} /* namespace alica */
