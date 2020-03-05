#ifndef Go2RandomPosition_H_
#define Go2RandomPosition_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1542881969548) ENABLED START*/  // Add additional includes here
/*PROTECTED REGION END*/
namespace alica {
class Go2RandomPosition : public DomainBehaviour {
public:
    Go2RandomPosition();
    virtual ~Go2RandomPosition();
    virtual void run(void* msg);
    /*PROTECTED REGION ID(pub1542881969548) ENABLED START*/  // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1542881969548) ENABLED START*/  // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1542881969548) ENABLED START*/  // Add additional private methods here
        /*PROTECTED REGION END*/};
        } /* namespace alica */

#endif /* Go2RandomPosition_H_ */
