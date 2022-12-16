#pragma once

#include <alica/DomainCondition.h>
#include <alica/DomainPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1889749086610694100) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1889749086610694100) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
class Move1889749086610694100 : public DomainPlan
{
public:
    Move1889749086610694100(PlanContext& context);
    virtual ~Move1889749086610694100();
    /*PROTECTED REGION ID(pub1889749086610694100) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    /*PROTECTED REGION ID(pro1889749086610694100) ENABLED START*/
    // Override these methods for your use case
    // virtual void run() override;
    // virtual void onInit() override;
    // virtual void onTerminate() override;
    // Add/Override protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1889749086610694100) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};

class UtilityFunction1889749086610694100 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class RunTimeCondition1288817888979746811 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* worldModels);
};
} /* namespace alica */
