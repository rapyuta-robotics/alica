#include "RuntimeConditionPlan1418042806575.h"
/*PROTECTED REGION ID(eph1418042806575) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  RuntimeConditionPlan (1418042806575)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1418042806577)
//
// States:
//   - RuntimeConditionTest (1418042806576)
RuntimeConditionPlan1418042806575::RuntimeConditionPlan1418042806575()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1418042806575) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
RuntimeConditionPlan1418042806575::~RuntimeConditionPlan1418042806575()
{
    /*PROTECTED REGION ID(dcon1418042806575) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): , (Comment) :

/**
 * Available Vars:
 */
bool RunTimeCondition1418042967134::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1418042967134) ENABLED START*/
    std::cout << "The RunTimeCondition 1418042967134 in Plan 'RuntimeConditionPlan' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1418042806577
 */
std::shared_ptr<UtilityFunction> UtilityFunction1418042806575::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1418042806575) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1418042806575) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
