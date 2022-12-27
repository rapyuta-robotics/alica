#pragma once

#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <supplementary_tests/DomainCondition.h>
#include <supplementary_tests/DomainPlan.h>
/*PROTECTED REGION ID(incl1524452721452) ENABLED START*/
// Add inlcudes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1524452721452) ENABLED START*/
// Add other things here
/*PROTECTED REGION END*/
class VHMaster1524452721452 : public DomainPlan
{
public:
    VHMaster1524452721452(PlanContext& context);
    virtual ~VHMaster1524452721452();
    /*PROTECTED REGION ID(pub1524452721452) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1524452721452) ENABLED START*/
    // Override these methods for your use case
    // virtual void run() override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1524452721452) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1524452721452 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class RunTimeCondition1524463006078 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
} /* namespace alica */
