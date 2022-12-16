#pragma once

#include <alica_tests/DomainBehaviour.h>
/*PROTECTED REGION ID(inc1402488939130) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class Tackle : public DomainBehaviour
{
public:
    Tackle(BehaviourContext& context);
    virtual ~Tackle();
    virtual void run();
    /*PROTECTED REGION ID(pub1402488939130) ENABLED START*/
    // Add additional public methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1402488939130) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv1402488939130) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
