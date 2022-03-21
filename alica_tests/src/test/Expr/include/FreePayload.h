#pragma once

#include "DomainBehaviour.h"
#include <engine/IAlicaWorldModel.h>
/*PROTECTED REGION ID(inc422054015709952219) ENABLED START*/
// Add additional includes here
#include "alica_tests/TaskInstantiationIntegrationWorldModel.h"
/*PROTECTED REGION END*/

namespace alica
{
class FreePayload : public DomainBehaviour
{
public:
    FreePayload(IAlicaWorldModel* wm);
    virtual ~FreePayload();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub422054015709952219) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro422054015709952219) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv422054015709952219) ENABLED START*/
    // Add additional private methods here
    alicaTests::TaskInstantiationIntegrationWorldModel* _worldModel;
    /*PROTECTED REGION END*/
};
} /* namespace alica */
