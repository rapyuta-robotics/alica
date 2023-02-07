#include <alica_tests/Behaviour/IncreaseCountByXBeh.h>
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

IncreaseCountByXBeh::IncreaseCountByXBeh(BehaviourContext& context)
        : DomainBehaviour(context)
{
    /*PROTECTED REGION ID(con1084111613399827667) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
IncreaseCountByXBeh::~IncreaseCountByXBeh()
{
    /*PROTECTED REGION ID(dcon1084111613399827667) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void IncreaseCountByXBeh::run()
{
    /*PROTECTED REGION ID(run1084111613399827667) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void IncreaseCountByXBeh::initialiseParameters()
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
