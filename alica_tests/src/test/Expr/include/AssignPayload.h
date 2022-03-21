#pragma once

#include "DomainBehaviour.h"
#include <engine/IAlicaWorldModel.h>
/*PROTECTED REGION ID(inc3826644292150922713) ENABLED START*/
// Add additional includes here
#include "alica_tests/TaskInstantiationIntegrationWorldModel.h"
/*PROTECTED REGION END*/

namespace alica
{
class AssignPayload : public DomainBehaviour
{
public:
    AssignPayload(IAlicaWorldModel* wm);
    virtual ~AssignPayload();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub3826644292150922713) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro3826644292150922713) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv3826644292150922713) ENABLED START*/
    // Add additional private methods here
    alicaTests::TaskInstantiationIntegrationWorldModel* _worldModel;
    /*PROTECTED REGION END*/
};
} /* namespace alica */
