#ifndef Joystick_H_
#define Joystick_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1421854685673) ENABLED START*/ // Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
class Joystick : public DomainBehaviour
{
public:
    Joystick();
    virtual ~Joystick();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub1421854685673) ENABLED START*/ // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1421854685673) ENABLED START*/ // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1421854685673) ENABLED START*/ // Add additional private methods here
        /*PROTECTED REGION END*/};
        } /* namespace alica */

#endif /* Joystick_H_ */
