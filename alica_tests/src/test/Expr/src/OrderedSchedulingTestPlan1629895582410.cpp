#include <alica_tests/OrderedSchedulingTestPlan1629895582410.h>
/*PROTECTED REGION ID(eph1629895582410) ENABLED START*/
// Add additional options here
#include <alica_tests/test_sched_world_model.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  OrderedSchedulingTestPlan (1629895582410)
//
// Tasks:
//   - SchedulerTestEntrypoint (1613371619454) (Entrypoint: 1629895738193)
//
// States:
//   - PlanA (1629895681520)
//   - PlanB (1629895684249)
OrderedSchedulingTestPlan1629895582410::OrderedSchedulingTestPlan1629895582410(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1629895582410) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
OrderedSchedulingTestPlan1629895582410::~OrderedSchedulingTestPlan1629895582410()
{
    /*PROTECTED REGION ID(dcon1629895582410) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: SchedulerTestEntrypoint  -> EntryPoint-ID: 1629895738193
 */
std::shared_ptr<UtilityFunction> UtilityFunction1629895582410::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1629895582410) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1629895582410) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
