#include <alica_tests/Behaviour/SuccessNormal.h>
#include <memory>

/*PROTECTED REGION ID(inccpp2951008684180289642) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars2951008684180289642) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

SuccessNormal::SuccessNormal(BehaviourContext& context)
        : DomainBehaviour(context)
{
    /*PROTECTED REGION ID(con2951008684180289642) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SuccessNormal::~SuccessNormal()
{
    /*PROTECTED REGION ID(dcon2951008684180289642) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void SuccessNormal::run()
{
    /*PROTECTED REGION ID(run2951008684180289642) ENABLED START*/
    // Add additional options here
    setSuccess();
    /*PROTECTED REGION END*/
}
void SuccessNormal::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters2951008684180289642) ENABLED START*/
    // Add additional options here

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods2951008684180289642) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
