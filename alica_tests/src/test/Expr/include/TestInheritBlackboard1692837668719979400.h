#pragma once

#include "DomainCondition.h"
#include "DomainPlan.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaWorldModel.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1692837668719979400) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1692837668719979400) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class TestInheritBlackboard1692837668719979400 : public DomainPlan
{
public:
    TestInheritBlackboard1692837668719979400(IAlicaWorldModel* wm);
    virtual ~TestInheritBlackboard1692837668719979400();
    /*PROTECTED REGION ID(pub1692837668719979400) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1692837668719979400) ENABLED START*/
    // Add/Override protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1692837668719979400) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1692837668719979400 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */