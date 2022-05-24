#pragma once

#include <alica_tests/DomainBehaviour.h>
/*PROTECTED REGION ID(inc1532424188199) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class AlwaysFail : public DomainBehaviour
{
public:
    AlwaysFail(BehaviourContext& context);
    virtual ~AlwaysFail();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub1532424188199) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1532424188199) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1532424188199) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
