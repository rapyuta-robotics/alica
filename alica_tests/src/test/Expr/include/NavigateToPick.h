#pragma once

#include "DomainBehaviour.h"
#include <engine/IAlicaWorldModel.h>
/*PROTECTED REGION ID(inc4505472195947429717) ENABLED START*/
// Add additional includes here
#include "alica_tests/TaskInstantiationIntegrationWorldModel.h"
/*PROTECTED REGION END*/

namespace alica
{
class NavigateToPick : public DomainBehaviour
{
public:
    NavigateToPick(IAlicaWorldModel* wm);
    virtual ~NavigateToPick();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub4505472195947429717) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro4505472195947429717) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv4505472195947429717) ENABLED START*/
    // Add additional private methods here
    alicaTests::TaskInstantiationIntegrationWorldModel* _worldModel;
    uint64_t _agentId;
    /*PROTECTED REGION END*/
};
} /* namespace alica */
