#include <alica_tests/behaviours/State2Behaviour.h>
#include <memory>

/*PROTECTED REGION ID(inccpp2219945377054027027) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars2219945377054027027) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

State2Behaviour::State2Behaviour(BehaviourContext& context)
        : DomainBehaviour(context)
{
    /*PROTECTED REGION ID(con2219945377054027027) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
State2Behaviour::~State2Behaviour()
{
    /*PROTECTED REGION ID(dcon2219945377054027027) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void State2Behaviour::run()
{
    /*PROTECTED REGION ID(run2219945377054027027) ENABLED START*/
    // Add additional options here
    int64_t callCounter = LockedBlackboardRW(*getBlackboard()).get<int64_t>("callCount");
    ++callCounter;
    LockedBlackboardRW(*getBlackboard()).set("callCount", callCounter);
    /*PROTECTED REGION END*/
}
void State2Behaviour::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters2219945377054027027) ENABLED START*/
    // Add additional options here
    LockedBlackboardRW(*getBlackboard()).set("callCount", 0);

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods2219945377054027027) ENABLED START*/
// Add additional options here
int State2Behaviour::getCallCounter()
{
    return LockedBlackboardRW(*getBlackboard()).get<int64_t>("callCount");
}
/*PROTECTED REGION END*/

} /* namespace alica */
