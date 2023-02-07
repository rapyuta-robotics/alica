#pragma once

#include <alica_tests/DomainBehaviour.h>
/*PROTECTED REGION ID(inc1084111613399827667) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class IncreaseCountByX : public DomainBehaviour
{
public:
    IncreaseCountByX(BehaviourContext& context);
    virtual ~IncreaseCountByX();
    virtual void run();
    /*PROTECTED REGION ID(pub1084111613399827667) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1084111613399827667) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1084111613399827667) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
