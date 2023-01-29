#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl2504351804499332310) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth2504351804499332310) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class RunBehaviourInSimplePlan2504351804499332310 : public DomainPlan
{
public:
    RunBehaviourInSimplePlan2504351804499332310(PlanContext& context);
    virtual ~RunBehaviourInSimplePlan2504351804499332310();
    /*PROTECTED REGION ID(pub2504351804499332310) ENABLED START*/
    // Add additional public methods here
    int64_t getRunTimeConditionCounter();
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro2504351804499332310) ENABLED START*/
    // Override these methods for your use case
    // virtual void run() override;
    virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv2504351804499332310) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction2504351804499332310 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class RunTimeCondition4404788800584486714 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition122747038863060590 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
} /* namespace alica */
