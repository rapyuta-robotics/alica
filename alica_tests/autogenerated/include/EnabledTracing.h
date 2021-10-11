#pragma once

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc3606794787493300754) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class EnabledTracing : public DomainBehaviour
{
public:
    EnabledTracing();
    virtual ~EnabledTracing();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub3606794787493300754) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro3606794787493300754) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv3606794787493300754) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
