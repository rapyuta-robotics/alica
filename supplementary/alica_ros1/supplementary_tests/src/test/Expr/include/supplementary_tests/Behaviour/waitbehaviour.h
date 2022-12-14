#pragma once

#include <supplementary_tests/DomainBehaviour.h>
/*PROTECTED REGION ID(inc1234) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class waitbehaviour : public DomainBehaviour
{
public:
    waitbehaviour(BehaviourContext& context);
    virtual ~waitbehaviour();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub1234) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1234) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1234) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
