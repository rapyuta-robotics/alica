#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1629895837159) ENABLED START*/
// Add additional includes here
#include <atomic>
namespace alica_test
{
class SchedWM;
}
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1629895837159) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class PlanA1629895837159 : public DomainPlan
{
public:
    PlanA1629895837159(PlanContext& context);
    virtual ~PlanA1629895837159();
    /*PROTECTED REGION ID(pub1629895837159) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1629895837159) ENABLED START*/
    // Override these methods for your use case
    virtual void run(void* msg) override;
    virtual void onInit() override;
    virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1629895837159) ENABLED START*/
    // Add additional private methods here
    std::atomic<bool> _inRunContext;
    alica_test::SchedWM* _wm;
    /*PROTECTED REGION END*/
};

class UtilityFunction1629895837159 : public BasicUtilityFunction
{
public:
    UtilityFunction1629895837159(IAlicaLogger& logger);
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
