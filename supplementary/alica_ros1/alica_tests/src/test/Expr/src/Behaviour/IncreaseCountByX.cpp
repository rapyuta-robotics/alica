#include <alica_tests/Behaviour/IncreaseCountByX.h>
#include <memory>

/*PROTECTED REGION ID(inccpp1084111613399827667) ENABLED START*/
// Add additional includes here
#include <alica/test/CounterClass.h>
#include <engine/blackboard/Blackboard.h>
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars1084111613399827667) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

IncreaseCountByX::IncreaseCountByX(BehaviourContext& context)
        : DomainBehaviour(context)
{
    /*PROTECTED REGION ID(con1084111613399827667) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
IncreaseCountByX::~IncreaseCountByX()
{
    /*PROTECTED REGION ID(dcon1084111613399827667) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void IncreaseCountByX::run()
{
    /*PROTECTED REGION ID(run1084111613399827667) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void IncreaseCountByX::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters1084111613399827667) ENABLED START*/
    // Add additional options here
    CounterClass::called += LockedBlackboardRO(*getBlackboard()).get<uint64_t>("increaseBy");

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1084111613399827667) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
