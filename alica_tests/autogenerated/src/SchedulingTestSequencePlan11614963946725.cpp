#include "SchedulingTestSequencePlan11614963946725.h"
/*PROTECTED REGION ID(eph1614963946725) ENABLED START*/
// Add additional options here
#include "CounterClass.h"
/*PROTECTED REGION END*/

namespace alica
{
// Plan:SchedulingTestSequencePlan11614963946725
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
 * Outgoing transition:
 *   - Name: 1614964566531, ConditionString: , Comment: MISSING_COMMENT
 *
 * Abstract plans in current state:
 *
 * Tasks in plan:
 *   - SchedulerTestSubPlanEntrypoint (1613372009777) (Entrypoint: 1614963977287)
 *
 * States in plan:
 *   - InitSequencePlan1 (1614963979424)
 *   - InitSequenceSubPlan1 (1614964540694)
 *   - InitSequenceSubPlan2 (1614964541828)
 *   - InitSequenceSubPlan3 (1614964542678)
 *   - TerminateSequenceSubPlan3 (1614964543300)
 *
 * Variables of precondition:
 */
bool PreCondition1614964566531::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1614964566530) ENABLED START*/
    return CounterClass::called == 2;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1614964572495, ConditionString: , Comment: MISSING_COMMENT
 *
 * Abstract plans in current state:
 *   - SchedulingTestSequenceSubPlan1 (1614964379654)
 *
 * Tasks in plan:
 *   - SchedulerTestSubPlanEntrypoint (1613372009777) (Entrypoint: 1614963977287)
 *
 * States in plan:
 *   - InitSequencePlan1 (1614963979424)
 *   - InitSequenceSubPlan1 (1614964540694)
 *   - InitSequenceSubPlan2 (1614964541828)
 *   - InitSequenceSubPlan3 (1614964542678)
 *   - TerminateSequenceSubPlan3 (1614964543300)
 *
 * Variables of precondition:
 */
bool PreCondition1614964572495::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1614964572494) ENABLED START*/
    return CounterClass::called == 3;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1614964575553, ConditionString: , Comment: MISSING_COMMENT
 *
 * Abstract plans in current state:
 *   - SchedulingTestSequenceSubPlan2 (1614964444419)
 *
 * Tasks in plan:
 *   - SchedulerTestSubPlanEntrypoint (1613372009777) (Entrypoint: 1614963977287)
 *
 * States in plan:
 *   - InitSequencePlan1 (1614963979424)
 *   - InitSequenceSubPlan1 (1614964540694)
 *   - InitSequenceSubPlan2 (1614964541828)
 *   - InitSequenceSubPlan3 (1614964542678)
 *   - TerminateSequenceSubPlan3 (1614964543300)
 *
 * Variables of precondition:
 */
bool PreCondition1614964575553::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1614964575552) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1614964578016, ConditionString: , Comment: MISSING_COMMENT
 *
 * Abstract plans in current state:
 *   - SchedulingTestSequenceSubPlan3 (1614964478264)
 *
 * Tasks in plan:
 *   - SchedulerTestSubPlanEntrypoint (1613372009777) (Entrypoint: 1614963977287)
 *
 * States in plan:
 *   - InitSequencePlan1 (1614963979424)
 *   - InitSequenceSubPlan1 (1614964540694)
 *   - InitSequenceSubPlan2 (1614964541828)
 *   - InitSequenceSubPlan3 (1614964542678)
 *   - TerminateSequenceSubPlan3 (1614964543300)
 *
 * Variables of precondition:
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
