#pragma once

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1629895911592) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class BehBAA : public DomainBehaviour
{
public:
    BehBAA();
    virtual ~BehBAA();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub1629895911592) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1629895911592) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1629895911592) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
