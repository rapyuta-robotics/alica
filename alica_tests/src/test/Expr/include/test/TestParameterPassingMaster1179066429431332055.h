#pragma once

#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <test/DomainCondition.h>
#include <test/DomainPlan.h>
/*PROTECTED REGION ID(incl1179066429431332055) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1179066429431332055) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class TestParameterPassingMaster1179066429431332055 : public DomainPlan
{
public:
    TestParameterPassingMaster1179066429431332055(PlanContext& context);
    virtual ~TestParameterPassingMaster1179066429431332055();
    /*PROTECTED REGION ID(pub1179066429431332055) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1179066429431332055) ENABLED START*/
    // Add/Override protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1179066429431332055) ENABLED START*/
    virtual void onInit() override;
    /*PROTECTED REGION END*/
};

class UtilityFunction1179066429431332055 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
