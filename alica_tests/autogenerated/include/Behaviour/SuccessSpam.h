#pragma once

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1522377401286) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class SuccessSpam : public DomainBehaviour
{
public:
    SuccessSpam();
    virtual ~SuccessSpam();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub1522377401286) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1522377401286) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1522377401286) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
