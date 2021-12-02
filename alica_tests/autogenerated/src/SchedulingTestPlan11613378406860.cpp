#include "SchedulingTestPlan11613378406860.h"
/*PROTECTED REGION ID(eph1613378406860) ENABLED START*/
// Add additional options here
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
SchedulingTestPlan11613378406860::SchedulingTestPlan11613378406860()
        : DomainPlan()
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
bool PreCondition1614960055821::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1614960055819) ENABLED START*/
    std::cout << "The PreCondition 1614960055821 in Transition 'FromDefault NameTo Default Name' is not implement yet!" << std::endl;
    return false;
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
bool PreCondition1614960063843::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1614960063842) ENABLED START*/
    std::cout << "The PreCondition 1614960063843 in Transition 'FromDefault NameTo Default Name' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1613378406860) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
