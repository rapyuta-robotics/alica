#pragma once

#include "DomainBehaviour.h"
#include <engine/IAlicaWorldModel.h>
/*PROTECTED REGION ID(inc2580816776008671737) ENABLED START*/
// Add additional includes here
#include "alica_tests/TaskInstantiationIntegrationWorldModel.h"
/*PROTECTED REGION END*/

namespace alica
{
class Pick : public DomainBehaviour
{
public:
    Pick(IAlicaWorldModel* wm);
    virtual ~Pick();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub2580816776008671737) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro2580816776008671737) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv2580816776008671737) ENABLED START*/
    // Add additional private methods here
    alicaTests::TaskInstantiationIntegrationWorldModel* _worldModel;
    uint64_t _agentId;
    /*PROTECTED REGION END*/
};
} /* namespace alica */
