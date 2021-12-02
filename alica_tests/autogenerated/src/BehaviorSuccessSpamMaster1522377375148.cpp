#include "BehaviorSuccessSpamMaster1522377375148.h"
/*PROTECTED REGION ID(eph1522377375148) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  BehaviorSuccessSpamMaster (1522377375148)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1522377375150)
//
// States:
//   - Normal (1522377375149)
//   - Dummy (1522377929290)
BehaviorSuccessSpamMaster1522377375148::BehaviorSuccessSpamMaster1522377375148()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1522377375148) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
BehaviorSuccessSpamMaster1522377375148::~BehaviorSuccessSpamMaster1522377375148()
{
    /*PROTECTED REGION ID(dcon1522377375148) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1522377375150
 */
std::shared_ptr<UtilityFunction> UtilityFunction1522377375148::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1522377375148) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
/**
 * Transition: MISSING_NAME (1522377944058)
 *   - Comment:
 *   - Source2Dest: Normal --> Dummy
 *
 * Precondition: MISSING_NAME (1522377944921)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Normal:
 *   - SuccessSpam (1522377401286)
 */
bool PreCondition1522377944921::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1522377944058) ENABLED START*/
    std::cout << "The PreCondition 1522377944921 in Transition 'MISSING_NAME' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Transition: MISSING_NAME (1522377945069)
 *   - Comment:
 *   - Source2Dest: Dummy --> Normal
 *
 * Precondition: MISSING_NAME (1522377946607)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Dummy:
 *   - SuccessSpam (1522377401286)
 */
bool PreCondition1522377946607::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1522377945069) ENABLED START*/
    std::cout << "The PreCondition 1522377946607 in Transition 'MISSING_NAME' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1522377375148) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
