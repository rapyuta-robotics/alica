#pragma once

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc4054297592460872311) ENABLED START*/
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
    /*PROTECTED REGION ID(pub4054297592460872311) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro4054297592460872311) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv4054297592460872311) ENABLED START*/
    // Add additional private methods here
    alica::Query _query;
    std::vector<double> _results;
    /*PROTECTED REGION END*/
};
} /* namespace alica */
