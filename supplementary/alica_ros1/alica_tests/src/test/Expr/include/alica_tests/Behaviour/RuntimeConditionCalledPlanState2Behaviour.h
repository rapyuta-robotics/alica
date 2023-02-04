#pragma once

#include <alica_tests/DomainBehaviour.h>
/*PROTECTED REGION ID(inc704908733785015826) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class RuntimeConditionCalledPlanState2Behaviour : public DomainBehaviour
{
public:
    RuntimeConditionCalledPlanState2Behaviour(BehaviourContext& context);
    virtual ~RuntimeConditionCalledPlanState2Behaviour();
    virtual void run();
    /*PROTECTED REGION ID(pub704908733785015826) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro704908733785015826) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv704908733785015826) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
