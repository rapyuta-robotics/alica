#pragma once

#include <alica_tests/DomainBehaviour.h>
/*PROTECTED REGION ID(inc3505111757300078074) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class SuccessDummy : public DomainBehaviour
{
public:
    SuccessDummy(BehaviourContext& context);
    virtual ~SuccessDummy();
    virtual void run();
    /*PROTECTED REGION ID(pub3505111757300078074) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro3505111757300078074) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv3505111757300078074) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */