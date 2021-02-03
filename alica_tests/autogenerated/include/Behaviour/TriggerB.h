#pragma once

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1428508316905) ENABLED START*/
// Add additional includes here
#include "TestWorldModel.h"
#include <essentials/ITrigger.hpp>

/*PROTECTED REGION END*/

namespace alica
{
class TriggerB : public DomainBehaviour
{
public:
    TriggerB();
    virtual ~TriggerB();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub1428508316905) ENABLED START*/
    // Add additional public methods here
    int callCounter;
    int initCounter;
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1428508316905) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1428508316905) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
