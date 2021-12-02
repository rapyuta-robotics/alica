#include "MultiAgentTestMaster1413200842973.h"
/*PROTECTED REGION ID(eph1413200842973) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  MultiAgentTestMaster (1413200842973)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1413200842975)
//
// States:
//   - Init (1413200842974)
//   - Start (1413201213955)
//   - Finished (1413201380359)
MultiAgentTestMaster1413200842973::MultiAgentTestMaster1413200842973()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1413200842973) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
MultiAgentTestMaster1413200842973::~MultiAgentTestMaster1413200842973()
{
    /*PROTECTED REGION ID(dcon1413200842973) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1413200842975
 */
std::shared_ptr<UtilityFunction> UtilityFunction1413200842973::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1413200842973) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
/**
 * Transition: 1413201226246 (1413201226246)
 *   - Comment:
 *   - Source2Dest: Init --> Start
 *
 * Precondition: 1413201227586 (1413201227586)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Init:
 */
bool PreCondition1413201227586::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1413201226246) ENABLED START*/
    std::cout << "The PreCondition 1413201227586 in Transition '1413201226246' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Transition: 1413201388722 (1413201388722)
 *   - Comment:
 *   - Source2Dest: Start --> Finished
 *
 * Precondition: 1413201389955 (1413201389955)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Start:
 *   - MultiAgentTestPlan (1413200862180)
 */
bool PreCondition1413201389955::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1413201388722) ENABLED START*/
    std::cout << "The PreCondition 1413201389955 in Transition '1413201388722' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1413200842973) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
