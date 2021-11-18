#pragma once

#include "DomainBehaviour.h"
#include <engine/IAlicaWorldModel.h>
/*PROTECTED REGION ID(inc1629895901559) ENABLED START*/
// Add additional includes here
#include <atomic>

namespace alica_test
{
class SchedWM;
}
/*PROTECTED REGION END*/

namespace alica
{
class BehAAA : public DomainBehaviour
{
public:
    BehAAA(IAlicaWorldModel* wm);
    virtual ~BehAAA();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub1629895901559) ENABLED START*/
    // Add additional protected methods here
    int runCount;
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1629895901559) ENABLED START*/
    // Add additional protected methods here
    virtual void onTermination();
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1629895901559) ENABLED START*/
    // Add additional private methods here
    std::atomic<bool> _inRunContext;
    alica_test::SchedWM* _wm;
    /*PROTECTED REGION END*/
};
} /* namespace alica */
