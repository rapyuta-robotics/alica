#pragma once

#include <alica_tests/DomainBehaviour.h>
/*PROTECTED REGION ID(inc2580864383983173919) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class RuntimeConditionCalledPlanState1Behaviour : public DomainBehaviour
{
public:
    RuntimeConditionCalledPlanState1Behaviour(BehaviourContext& context);
    virtual ~RuntimeConditionCalledPlanState1Behaviour();
    virtual void run();
    /*PROTECTED REGION ID(pub2580864383983173919) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro2580864383983173919) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv2580864383983173919) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
