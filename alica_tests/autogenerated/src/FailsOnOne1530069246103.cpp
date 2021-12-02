#include "FailsOnOne1530069246103.h"
/*PROTECTED REGION ID(eph1530069246103) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  FailsOnOne (1530069246103)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1530069246105)
//
// States:
//   - NewState (1530069246104)
FailsOnOne1530069246103::FailsOnOne1530069246103()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1530069246103) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
FailsOnOne1530069246103::~FailsOnOne1530069246103()
{
    /*PROTECTED REGION ID(dcon1530069246103) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): , (Comment) : Is not set 1

/**
 * Available Vars:
 */
bool RunTimeCondition1530069251117::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1530069251117) ENABLED START*/
    std::cout << "The RunTimeCondition 1530069251117 in Plan 'FailsOnOne' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1530069246105
 */
std::shared_ptr<UtilityFunction> UtilityFunction1530069246103::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1530069246103) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1530069246103) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
