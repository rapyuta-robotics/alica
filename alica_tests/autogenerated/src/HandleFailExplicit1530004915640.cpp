#include "HandleFailExplicit1530004915640.h"
/*PROTECTED REGION ID(eph1530004915640) ENABLED START*/
// Add additional using directives here
#include <alica_tests/SimpleSwitches.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  HandleFailExplicit (1530004915640)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1530004915642)
//
// States:
//   - A (1530004915641)
//   - B (1530004973591)
//   - C (1530004975275)
//   - D (1532424087894)
//   - E (1532424097662)
HandleFailExplicit1530004915640::HandleFailExplicit1530004915640()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1530004915640) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
HandleFailExplicit1530004915640::~HandleFailExplicit1530004915640()
{
    /*PROTECTED REGION ID(dcon1530004915640) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

void HandleFailExplicit1530004915640::run(void* msg)
{
    /*PROTECTED REGION ID(runHandleFailExplicit1530004915640) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1530004915642
 */
std::shared_ptr<UtilityFunction> UtilityFunction1530004915640::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1530004915640) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
/**
 * Transition: MISSING_NAME (1530004992551)
 *   - Comment: From A to B, isset(0)
 *   - Source2Dest: A --> B
 *
 * Precondition: MISSING_NAME (1530004993493)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in A:
 */
bool PreCondition1530004993493::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1530004992551) ENABLED START*/
    return SimpleSwitches::isSet(0);
    /*PROTECTED REGION END*/
}
/**
 * Transition: MISSING_NAME (1530004993680)
 *   - Comment: AnyChildFail
 *   - Source2Dest: B --> C
 *
 * Precondition: MISSING_NAME (1530004994611)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in B:
 *   - FailsOnOne (1530069246103)
 */
bool PreCondition1530004994611::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1530004993680) ENABLED START*/
    return rp->isAnyChildStatus(PlanStatus::Failed);
    /*PROTECTED REGION END*/
}
/**
 * Transition: MISSING_NAME (1532424092280)
 *   - Comment: C to D, isset(2)
 *   - Source2Dest: C --> D
 *
 * Precondition: MISSING_NAME (1532424093178)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in C:
 */
bool PreCondition1532424093178::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1532424092280) ENABLED START*/
    return SimpleSwitches::isSet(2);
    /*PROTECTED REGION END*/
}
/**
 * Transition: MISSING_NAME (1532424112331)
 *   - Comment: AnyChildFail
 *   - Source2Dest: D --> E
 *
 * Precondition: MISSING_NAME (1532424113475)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in D:
 *   - AlwaysFail (1532424188199)
 */
bool PreCondition1532424113475::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1532424112331) ENABLED START*/
    return rp->isAnyChildStatus(PlanStatus::Failed);
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1530004915640) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
