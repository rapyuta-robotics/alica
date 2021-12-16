#include "BehaviorSuccessSpamMaster1522377375148.h"
/*PROTECTED REGION ID(eph1522377375148) ENABLED START*/
// Add additional using directives here
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
BehaviorSuccessSpamMaster1522377375148::BehaviorSuccessSpamMaster1522377375148(IAlicaWorldModel* wm)
        : DomainPlan(wm)
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
bool PreCondition1522377944921::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1522377944058) ENABLED START*/
    bool ok = rp->isAnyChildStatus(alica::PlanStatus::Success);
    // std::cout << "A "<<ok <<std::endl;
    return ok;
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
bool PreCondition1522377946607::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
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
