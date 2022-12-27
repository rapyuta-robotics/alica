#pragma once

#include <alica_tests/DomainBehaviour.h>
/*PROTECTED REGION ID(inc1402488696205) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class MidFieldStandard : public DomainBehaviour
{
public:
    MidFieldStandard(BehaviourContext& context);
    virtual ~MidFieldStandard();
    virtual void run();
    /*PROTECTED REGION ID(pub1402488696205) ENABLED START*/
    // Add additional public methods here
    int callCounter;
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1402488696205) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1402488696205) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
