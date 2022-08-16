#pragma once

#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <supplementary_tests/DomainCondition.h>
#include <supplementary_tests/DomainPlan.h>
/*PROTECTED REGION ID(incl1479556022226) ENABLED START*/
// Add inlcudes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1479556022226) ENABLED START*/
// Add other things here
/*PROTECTED REGION END*/
class ProblemBuildingMaster1479556022226 : public DomainPlan
{
public:
    ProblemBuildingMaster1479556022226(PlanContext& context);
    virtual ~ProblemBuildingMaster1479556022226();
    /*PROTECTED REGION ID(pub1479556022226) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1479556022226) ENABLED START*/
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1479556022226) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1479556022226 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
