#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1418042656594) ENABLED START*/
// Add inlcudes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1418042656594) ENABLED START*/
// Add other things here
/*PROTECTED REGION END*/
class MasterPlanTestConditionPlanType1418042656594 : public DomainPlan
{
public:
    MasterPlanTestConditionPlanType1418042656594(PlanContext& context);
    virtual ~MasterPlanTestConditionPlanType1418042656594();
    /*PROTECTED REGION ID(pub1418042656594) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1418042656594) ENABLED START*/
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1418042656594) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1418042656594 : public BasicUtilityFunction
{
public:
    UtilityFunction1418042656594();
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
