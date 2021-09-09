#include "SchedulingTestSequencePlan11614963946725.h"
/*PROTECTED REGION ID(eph1614963946725) ENABLED START*/
// Add additional options here
#include <alica_tests/CounterClass.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  SchedulingTestSequencePlan1 (1614963946725)
//
// Tasks:
//   - SchedulerTestSubPlanEntrypoint (1613372009777) (Entrypoint: 1614963977287)
//
// States:
//   - InitSequencePlan1 (1614963979424)
//   - InitSequenceSubPlan1 (1614964540694)
//   - InitSequenceSubPlan2 (1614964541828)
//   - InitSequenceSubPlan3 (1614964542678)
//   - TerminateSequenceSubPlan3 (1614964543300)
SchedulingTestSequencePlan11614963946725::SchedulingTestSequencePlan11614963946725()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1614963946725) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SchedulingTestSequencePlan11614963946725::~SchedulingTestSequencePlan11614963946725()
{
    /*PROTECTED REGION ID(dcon1614963946725) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: SchedulerTestSubPlanEntrypoint  -> EntryPoint-ID: 1614963977287
 */
std::shared_ptr<UtilityFunction> UtilityFunction1614963946725::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1614963946725) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
/**
 * Transition: FromInitSequencePlan1To InitSequenceSubPlan1 (1614964566530)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: InitSequencePlan1 --> InitSequenceSubPlan1
 *
 * Precondition: 1614964566531 (1614964566531)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in InitSequencePlan1:
 */
bool PreCondition1614964566531::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1614964566530) ENABLED START*/
    return CounterClass::called == 2;
    /*PROTECTED REGION END*/
}
/**
 * Transition: FromInitSequenceSubPlan1To InitSequenceSubPlan2 (1614964572494)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: InitSequenceSubPlan1 --> InitSequenceSubPlan2
 *
 * Precondition: 1614964572495 (1614964572495)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in InitSequenceSubPlan1:
 *   - SchedulingTestSequenceSubPlan1 (1614964379654)
 */
bool PreCondition1614964572495::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1614964572494) ENABLED START*/
    return CounterClass::called == 3;
    /*PROTECTED REGION END*/
}
/**
 * Transition: FromInitSequenceSubPlan2To InitSequenceSubPlan3 (1614964575552)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: InitSequenceSubPlan2 --> InitSequenceSubPlan3
 *
 * Precondition: 1614964575553 (1614964575553)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in InitSequenceSubPlan2:
 *   - SchedulingTestSequenceSubPlan2 (1614964444419)
 */
bool PreCondition1614964575553::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1614964575552) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Transition: FromInitSequenceSubPlan3To TerminateSequenceSubPlan3 (1614964578015)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: InitSequenceSubPlan3 --> TerminateSequenceSubPlan3
 *
 * Precondition: 1614964578016 (1614964578016)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in InitSequenceSubPlan3:
 *   - SchedulingTestSequenceSubPlan3 (1614964478264)
 */
bool PreCondition1614964578016::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1614964578015) ENABLED START*/
    return CounterClass::called == 4;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1614963946725) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
