#pragma once

#include "DomainBehaviour.h"
#include <engine/IAlicaWorldModel.h>
/*PROTECTED REGION ID(inc831400441334251602) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class TestParameterPassingBehaviour : public DomainBehaviour
{
public:
    TestParameterPassingBehaviour(IAlicaWorldModel* wm);
    virtual ~TestParameterPassingBehaviour();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub831400441334251602) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro831400441334251602) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv831400441334251602) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
