#pragma once

#include "DomainBehaviour.h"
#include <engine/IAlicaWorldModel.h>
/*PROTECTED REGION ID(inc3009473645416620380) ENABLED START*/
// Add additional includes here
#include "alica_tests/TaskInstantiationIntegrationWorldModel.h"
/*PROTECTED REGION END*/

namespace alica
{
class Drop : public DomainBehaviour
{
public:
    Drop(IAlicaWorldModel* wm);
    virtual ~Drop();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub3009473645416620380) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro3009473645416620380) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv3009473645416620380) ENABLED START*/
    // Add additional private methods here
    alicaTests::TaskInstantiationIntegrationWorldModel* _worldModel;
    uint64_t _agentId;
    /*PROTECTED REGION END*/
};
} /* namespace alica */
