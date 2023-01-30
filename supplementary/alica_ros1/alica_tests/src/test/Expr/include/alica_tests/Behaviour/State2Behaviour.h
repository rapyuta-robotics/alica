#pragma once

#include <alica_tests/DomainBehaviour.h>
/*PROTECTED REGION ID(inc2219945377054027027) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class State2Behaviour : public DomainBehaviour
{
public:
    State2Behaviour(BehaviourContext& context);
    virtual ~State2Behaviour();
    virtual void run();
    /*PROTECTED REGION ID(pub2219945377054027027) ENABLED START*/
    // Add additional protected methods here
    int getCallCounter();
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro2219945377054027027) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv2219945377054027027) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
