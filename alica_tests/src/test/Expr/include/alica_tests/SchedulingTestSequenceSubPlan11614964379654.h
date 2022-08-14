#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1614964379654) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1614964379654) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class SchedulingTestSequenceSubPlan11614964379654 : public DomainPlan
{
public:
    SchedulingTestSequenceSubPlan11614964379654(PlanContext& context);
    virtual ~SchedulingTestSequenceSubPlan11614964379654();
    /*PROTECTED REGION ID(pub1614964379654) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1614964379654) ENABLED START*/
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1614964379654) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1614964379654 : public BasicUtilityFunction
{
public:
    UtilityFunction1614964379654(IAlicaLogger& logger);
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
