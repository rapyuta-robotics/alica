#pragma once

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc55178365253414982) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class TestBehaviour : public DomainBehaviour
{
public:
    TestBehaviour();
    virtual ~TestBehaviour();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub55178365253414982) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro55178365253414982) ENABLED START*/
    // Add additional protected methods here
    virtual void onTermination();
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv55178365253414982) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
