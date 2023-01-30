#pragma once

#include <alica_tests/DomainBehaviour.h>
/*PROTECTED REGION ID(inc3563417394101512880) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class State1Behaviour : public DomainBehaviour
{
public:
    State1Behaviour(BehaviourContext& context);
    virtual ~State1Behaviour();
    virtual void run();
    /*PROTECTED REGION ID(pub3563417394101512880) ENABLED START*/
    // Add additional protected methods here
    int callCounter;
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro3563417394101512880) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv3563417394101512880) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
