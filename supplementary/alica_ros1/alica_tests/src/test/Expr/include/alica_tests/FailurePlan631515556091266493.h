#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl631515556091266493) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth631515556091266493) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class FailurePlan631515556091266493 : public DomainPlan
{
public:
    FailurePlan631515556091266493(PlanContext& context);
    virtual ~FailurePlan631515556091266493();
    /*PROTECTED REGION ID(pub631515556091266493) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro631515556091266493) ENABLED START*/
    // Override these methods for your use case
    // virtual void run() override;
    virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv631515556091266493) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction631515556091266493 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition4351457352348187886 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* wm);
};
class PreCondition2038762164340314344 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* wm);
};
} /* namespace alica */
