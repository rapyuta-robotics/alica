#pragma once

#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <supplementary_tests/DomainCondition.h>
#include <supplementary_tests/DomainPlan.h>
/*PROTECTED REGION ID(incl1417423757243) ENABLED START*/
// Add inlcudes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1417423757243) ENABLED START*/
// Add other things here
/*PROTECTED REGION END*/
class GSolverTestPlan1417423757243 : public DomainPlan
{
public:
    GSolverTestPlan1417423757243(PlanContext& context);
    virtual ~GSolverTestPlan1417423757243();
    /*PROTECTED REGION ID(pub1417423757243) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1417423757243) ENABLED START*/
    // Override these methods for your use case
    // virtual void run() override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1417423757243) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1417423757243 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class RunTimeCondition1417424512343 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* wm);
};
} /* namespace alica */
