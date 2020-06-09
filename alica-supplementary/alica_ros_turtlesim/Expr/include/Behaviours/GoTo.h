#pragma once

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1544160969061) ENABLED START*/
// Add additional includes here
#include <engine/constraintmodul/Query.h>
/*PROTECTED REGION END*/

namespace alica
{
class GoTo : public DomainBehaviour
{
public:
    GoTo();
    virtual ~GoTo();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub1544160969061) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1544160969061) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1544160969061) ENABLED START*/
    // Add additional private methods here
    alica::Query _query;
    std::vector<double> _results;
    /*PROTECTED REGION END*/
};
} /* namespace alica */
