#pragma once

#include "DomainBehaviour.h"
#include <engine/IAlicaWorldModel.h>
/*PROTECTED REGION ID(inc4459885370764933844) ENABLED START*/
// Add additional includes here
#include "alica_tests/TaskInstantiationIntegrationWorldModel.h"
/*PROTECTED REGION END*/

namespace alica
{
class NavigateToDrop : public DomainBehaviour
{
public:
    NavigateToDrop(IAlicaWorldModel* wm);
    virtual ~NavigateToDrop();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub4459885370764933844) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro4459885370764933844) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv4459885370764933844) ENABLED START*/
    // Add additional private methods here
    alicaTests::TaskInstantiationIntegrationWorldModel* _worldModel;
    uint64_t _agentId;
    /*PROTECTED REGION END*/
};
} /* namespace alica */
