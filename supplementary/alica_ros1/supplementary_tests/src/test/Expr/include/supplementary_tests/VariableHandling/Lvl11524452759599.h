#pragma once

#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <supplementary_tests/DomainCondition.h>
#include <supplementary_tests/DomainPlan.h>
/*PROTECTED REGION ID(incl1524452759599) ENABLED START*/
// Add inlcudes here
extern bool vhStartCondition;
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1524452759599) ENABLED START*/
// Add other things here

/*PROTECTED REGION END*/
class Lvl11524452759599 : public DomainPlan
{
public:
    Lvl11524452759599(PlanContext& context);
    virtual ~Lvl11524452759599();
    /*PROTECTED REGION ID(pub1524452759599) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1524452759599) ENABLED START*/
    // Override these methods for your use case
    // virtual void run() override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1524452759599) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1524452759599 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class RunTimeCondition1524453470580 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1524453491764 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
} /* namespace alica */
