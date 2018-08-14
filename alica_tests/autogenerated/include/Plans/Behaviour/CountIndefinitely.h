#ifndef CountIndefinitely_H_
#define CountIndefinitely_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1529456643148) ENABLED START*/ // Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
class CountIndefinitely : public DomainBehaviour
{
public:
    CountIndefinitely();
    virtual ~CountIndefinitely();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub1529456643148) ENABLED START*/ // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1529456643148) ENABLED START*/ // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
/*PROTECTED REGION ID(prv1529456643148) ENABLED START*/ // Add additional private methods here
        /*PROTECTED REGION END*/};
        } /* namespace alica */

#endif /* CountIndefinitely_H_ */
