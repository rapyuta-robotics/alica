#pragma once

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc4044546549214673470) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class DynamicTaskBehavior : public DomainBehaviour
{
public:
    DynamicTaskBehavior();
    virtual ~DynamicTaskBehavior();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub4044546549214673470) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro4044546549214673470) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv4044546549214673470) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
