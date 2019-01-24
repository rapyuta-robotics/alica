#ifndef NewBehaviour_H_
#define NewBehaviour_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1413810772726) ENABLED START*/ // Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
class NewBehaviour : public DomainBehaviour
{
public:
    NewBehaviour();
    virtual ~NewBehaviour();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub1413810772726) ENABLED START*/ // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1413810772726) ENABLED START*/ // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1413810772726) ENABLED START*/ // Add additional private methods here
        /*PROTECTED REGION END*/};
        } /* namespace alica */

#endif /* NewBehaviour_H_ */
