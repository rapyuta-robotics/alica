#ifndef NotToTrigger_H_
#define NotToTrigger_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1429017274116) ENABLED START*/ // Add additional includes here
#include "TestWorldModel.h"

#include <SystemConfig.h>
#include <essentials/ITrigger.h>
/*PROTECTED REGION END*/
namespace alica
{
class NotToTrigger : public DomainBehaviour
{
public:
    NotToTrigger();
    virtual ~NotToTrigger();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub1429017274116) ENABLED START*/ // Add additional public methods here
    int callCounter;
    int initCounter;
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1429017274116) ENABLED START*/ // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1429017274116) ENABLED START*/ // Add additional private methods here
        /*PROTECTED REGION END*/};
        } /* namespace alica */

#endif /* NotToTrigger_H_ */
