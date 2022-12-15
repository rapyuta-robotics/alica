#include <alica_tests/SchedulingTestSequencePlan11614963946725.h>
/*PROTECTED REGION ID(eph1614963946725) ENABLED START*/
// Add additional options here
#include <alica/test/CounterClass.h>
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
SchedulingTestSequencePlan11614963946725::SchedulingTestSequencePlan11614963946725(PlanContext& context)
        : DomainPlan(context)
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

/*PROTECTED REGION ID(methods1614963946725) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
