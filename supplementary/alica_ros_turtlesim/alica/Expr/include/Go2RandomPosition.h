#pragma once

#include "DomainBehaviour.h"
#include <engine/IAlicaWorldModel.h>
/*PROTECTED REGION ID(inc4085572422059465423) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class Go2RandomPosition : public DomainBehaviour
{
public:
    Go2RandomPosition(IAlicaWorldModel* wm);
    virtual ~Go2RandomPosition();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub4085572422059465423) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro4085572422059465423) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv4085572422059465423) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
