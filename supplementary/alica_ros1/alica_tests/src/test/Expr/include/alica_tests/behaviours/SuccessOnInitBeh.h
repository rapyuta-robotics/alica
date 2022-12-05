#pragma once

#include <alica_tests/DomainBehaviour.h>
/*PROTECTED REGION ID(inc3821787310391665935) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class SuccessOnInitBeh : public DomainBehaviour
{
public:
    SuccessOnInitBeh(BehaviourContext& context);
    virtual ~SuccessOnInitBeh();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub3821787310391665935) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro3821787310391665935) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv3821787310391665935) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
