#ifndef AlwaysFail_H_
#define AlwaysFail_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1532424188199) ENABLED START*/ // Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
class AlwaysFail : public DomainBehaviour
{
public:
    AlwaysFail();
    virtual ~AlwaysFail();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub1532424188199) ENABLED START*/ // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1532424188199) ENABLED START*/ // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1532424188199) ENABLED START*/ // Add additional private methods here
        /*PROTECTED REGION END*/};
        } /* namespace alica */

#endif /* AlwaysFail_H_ */
