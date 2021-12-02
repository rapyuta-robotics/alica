#pragma once

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1429017274116) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class NotToTrigger : public DomainBehaviour
{
public:
    NotToTrigger();
    virtual ~NotToTrigger();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub1429017274116) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1429017274116) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1429017274116) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
