#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1629895853508) ENABLED START*/
// Add additional includes here
namespace alica_test
{
class SchedWM;
}
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1629895853508) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class PlanB1629895853508 : public DomainPlan
{
public:
    PlanB1629895853508(PlanContext& context);
    virtual ~PlanB1629895853508();
    /*PROTECTED REGION ID(pub1629895853508) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1629895853508) ENABLED START*/
    // Override these methods for your use case
    // virtual void run() override;
    virtual void onInit() override;
    virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1629895853508) ENABLED START*/
    // Add additional private methods here
    std::shared_ptr<alica_test::SchedWM> _wm;
    /*PROTECTED REGION END*/
};

class UtilityFunction1629895853508 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
