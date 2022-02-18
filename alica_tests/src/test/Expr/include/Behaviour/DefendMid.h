#pragma once

#include "DomainBehaviour.h"
#include <engine/IAlicaWorldModel.h>
/*PROTECTED REGION ID(inc1402488730695) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class DefendMid : public DomainBehaviour
{
public:
    DefendMid(IAlicaWorldModel* wm);
    virtual ~DefendMid();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub1402488730695) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1402488730695) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1402488730695) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
