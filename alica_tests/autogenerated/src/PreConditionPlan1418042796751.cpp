#include "PreConditionPlan1418042796751.h"
/*PROTECTED REGION ID(eph1418042796751) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  PreConditionPlan (1418042796751)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1418042796753)
//
// States:
//   - PreConditionTest (1418042796752)
PreConditionPlan1418042796751::PreConditionPlan1418042796751()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1418042796751) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
PreConditionPlan1418042796751::~PreConditionPlan1418042796751()
{
    /*PROTECTED REGION ID(dcon1418042796751) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

// Check of PreCondition - (Name): NewPreCondition, (ConditionString): Test , (Comment) :

/**
 * Available Vars:
 */
bool PreCondition1418042929966::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1418042929966) ENABLED START*/
    std::cout << "The PreCondition 1418042929966 in Plan 'PreConditionPlan' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1418042796753
 */
std::shared_ptr<UtilityFunction> UtilityFunction1418042796751::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1418042796751) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1418042796751) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
