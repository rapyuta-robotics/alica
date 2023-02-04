#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl3213121947038933654) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth3213121947038933654) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class RuntimeConditionCalledPlan3213121947038933654 : public DomainPlan
{
public:
    RuntimeConditionCalledPlan3213121947038933654(PlanContext& context);
    virtual ~RuntimeConditionCalledPlan3213121947038933654();
    /*PROTECTED REGION ID(pub3213121947038933654) ENABLED START*/
    // Add additional public methods here
    int64_t getRunTimeConditionCounter();
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro3213121947038933654) ENABLED START*/
    // Override these methods for your use case
    // virtual void run() override;
    virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv3213121947038933654) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction3213121947038933654 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class RunTimeCondition4595076014383940051 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition778972856393262601 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
} /* namespace alica */
