#pragma once

#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <test/DomainCondition.h>
#include <test/DomainPlan.h>
/*PROTECTED REGION ID(incl3172561495666303184) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth3172561495666303184) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class ExecuteBehaviourInSubPlan3172561495666303184 : public DomainPlan
{
public:
    ExecuteBehaviourInSubPlan3172561495666303184(PlanContext& context);
    virtual ~ExecuteBehaviourInSubPlan3172561495666303184();
    /*PROTECTED REGION ID(pub3172561495666303184) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro3172561495666303184) ENABLED START*/
    // Override these methods for your use case
    // virtual void run(void* msg) override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add/Override protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv3172561495666303184) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction3172561495666303184 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1943478533524176732 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};
} /* namespace alica */
