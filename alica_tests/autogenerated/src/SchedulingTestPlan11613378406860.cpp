#include "SchedulingTestPlan11613378406860.h"
/*PROTECTED REGION ID(eph1613378406860) ENABLED START*/
// Add additional options here
#include <alica_tests/CounterClass.h>
#include <assert.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  SchedulingTestPlan1 (1613378406860)
//
// Tasks:
//   - SchedulerTestSubPlanEntrypoint (1613372009777) (Entrypoint: 1613378541158)
//
// States:
//   - InitPlan1 (1613378543512)
//   - TerminateSubPlans (1613977406218)
//   - InitSubPlans (1614960038398)
SchedulingTestPlan11613378406860::SchedulingTestPlan11613378406860(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1613378406860) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SchedulingTestPlan11613378406860::~SchedulingTestPlan11613378406860()
{
    /*PROTECTED REGION ID(dcon1613378406860) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: SchedulerTestSubPlanEntrypoint  -> EntryPoint-ID: 1613378541158
 */
std::shared_ptr<UtilityFunction> UtilityFunction1613378406860::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1613378406860) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
/**
 * Transition: FromDefault NameTo Default Name (1614960055819)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: InitPlan1 --> InitSubPlans
 *
 * Precondition: 1614960055821 (1614960055821)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in InitPlan1:
 */
bool PreCondition1614960055821::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1614960055819) ENABLED START*/
    return CounterClass::called == 2;
    /*PROTECTED REGION END*/
}
/**
 * Transition: FromDefault NameTo Default Name (1614960063842)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: InitSubPlans --> TerminateSubPlans
 *
 * Precondition: 1614960063843 (1614960063843)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in InitSubPlans:
 *   - SchedulingTestPlan2 (1613378423610)
 *   - SchedulingTestPlan3 (1613378433623)
 */
bool PreCondition1614960063843::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1614960063842) ENABLED START*/
    return CounterClass::called == 5;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1613378406860) ENABLED START*/
// Add additional options here
void SchedulingTestPlan11613378406860::onInit()
{
    CounterClass::called = 1;
}

void SchedulingTestPlan11613378406860::onTerminate()
{
    CounterClass::called += 1;
}
/*PROTECTED REGION END*/
} // namespace alica
