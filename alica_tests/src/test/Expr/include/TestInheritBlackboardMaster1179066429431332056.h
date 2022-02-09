#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaWorldModel.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1179066429431332056) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1179066429431332056) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class TestInheritBlackboardMaster1179066429431332056 : public DomainPlan
{
public:
    TestInheritBlackboardMaster1179066429431332056(IAlicaWorldModel* wm);
    virtual ~TestInheritBlackboardMaster1179066429431332056();
    /*PROTECTED REGION ID(pub1179066429431332056) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1179066429431332056) ENABLED START*/
    // Add/Override protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1179066429431332056) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1179066429431332056 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
