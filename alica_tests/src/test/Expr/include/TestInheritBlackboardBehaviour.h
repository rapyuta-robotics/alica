#pragma once

#include "DomainBehaviour.h"
#include <engine/IAlicaWorldModel.h>
/*PROTECTED REGION ID(inc831400441334251600) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class TestInheritBlackboardBehaviour : public DomainBehaviour
{
public:
    TestInheritBlackboardBehaviour(IAlicaWorldModel* wm);
    virtual ~TestInheritBlackboardBehaviour();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub831400441334251600) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro831400441334251600) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv831400441334251600) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
