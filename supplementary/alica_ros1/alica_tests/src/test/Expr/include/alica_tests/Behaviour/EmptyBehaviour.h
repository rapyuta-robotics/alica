#pragma once

#include <alica_tests/DomainBehaviour.h>
/*PROTECTED REGION ID(inc1625610857563) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class EmptyBehaviour : public DomainBehaviour
{
public:
    EmptyBehaviour(BehaviourContext& context);
    virtual ~EmptyBehaviour();
    virtual void run();
    /*PROTECTED REGION ID(pub1625610857563) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1625610857563) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1625610857563) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
