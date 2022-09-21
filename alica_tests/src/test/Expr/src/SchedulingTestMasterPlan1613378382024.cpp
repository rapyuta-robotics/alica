#include <alica_tests/SchedulingTestMasterPlan1613378382024.h>
/*PROTECTED REGION ID(eph1613378382024) ENABLED START*/
// Add additional options here
#include <alica_tests/CounterClass.h>
#include <alica_tests/test_sched_world_model.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  SchedulingTestMasterPlan (1613378382024)
//
// Tasks:
//   - SchedulerTestEntrypoint (1613371619454) (Entrypoint: 1615797283419)
//
// States:
//   - InitTest (1613378474109)
//   - EndTest (1613530614559)
//   - Default Name (1615797271229)
//   - Default Name (1615797319003)
//   - OrderedSchedulingTestPlan (1629895593451)
//   - ExecuteBehaviour (1206766322278521913)
//   - ExecuteBehaviourInSubPlan (3802371674214346622)
SchedulingTestMasterPlan1613378382024::SchedulingTestMasterPlan1613378382024(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1613378382024) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SchedulingTestMasterPlan1613378382024::~SchedulingTestMasterPlan1613378382024()
{
    /*PROTECTED REGION ID(dcon1613378382024) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: SchedulerTestEntrypoint  -> EntryPoint-ID: 1615797283419
 */
std::shared_ptr<UtilityFunction> UtilityFunction1613378382024::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1613378382024) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1613378382024) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
