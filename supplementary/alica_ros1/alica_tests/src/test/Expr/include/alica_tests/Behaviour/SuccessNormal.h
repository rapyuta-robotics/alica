#pragma once

#include <alica_tests/DomainBehaviour.h>
/*PROTECTED REGION ID(inc2951008684180289642) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
class SuccessNormal : public DomainBehaviour
{
public:
    SuccessNormal(BehaviourContext& context);
    virtual ~SuccessNormal();
    virtual void run();
    /*PROTECTED REGION ID(pub2951008684180289642) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro2951008684180289642) ENABLED START*/
    // Add additional protected methods here
    /*PROTECTED REGION END*/
private:
    /*PROTECTED REGION ID(prv2951008684180289642) ENABLED START*/
    // Add additional private methods here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
