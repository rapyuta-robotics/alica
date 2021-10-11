#pragma once

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc863651328966767832) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class DisabledTracing : public DomainBehaviour
{
public:
    DisabledTracing();
    virtual ~DisabledTracing();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub863651328966767832) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro863651328966767832) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv863651328966767832) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
