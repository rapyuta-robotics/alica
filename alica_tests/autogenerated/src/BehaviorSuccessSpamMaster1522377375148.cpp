#include "BehaviorSuccessSpamMaster1522377375148.h"
/*PROTECTED REGION ID(eph1522377375148) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:BehaviorSuccessSpamMaster1522377375148
BehaviorSuccessSpamMaster1522377375148::BehaviorSuccessSpamMaster1522377375148()
        : DomainPlan("BehaviorSuccessSpamMaster1522377375148")
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

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *   - SuccessSpam (1522377401286)
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1522377375150)
 *
 * States in plan:
 *   - Normal (1522377375149)
 *   - Dummy (1522377929290)
 *
 * Variables of precondition:
 */
bool PreCondition1522377944921::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1522377944058) ENABLED START*/
    bool ok = rp->isAnyChildStatus(alica::PlanStatus::Success);
    // std::cout << "A "<<ok <<std::endl;
    return ok;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *   - SuccessSpam (1522377401286)
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1522377375150)
 *
 * States in plan:
 *   - Normal (1522377375149)
 *   - Dummy (1522377929290)
 *
 * Variables of precondition:
 */
bool PreCondition1522377946607::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1522377945069) ENABLED START*/
    bool ok = rp->isAnyChildStatus(alica::PlanStatus::Success);
    // std::cout << "B "<<ok <<std::endl;
    return ok;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1522377375148) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
