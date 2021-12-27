#pragma once

#include "DomainBehaviour.h"
#include <engine/IAlicaWorldModel.h>
/*PROTECTED REGION ID(inc19516698765703926) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class DynamicTaskBehaviourLD : public DomainBehaviour
{
public:
    DynamicTaskBehaviourLD(IAlicaWorldModel* wm);
    virtual ~DynamicTaskBehaviourLD();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub19516698765703926) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro19516698765703926) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv19516698765703926) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
