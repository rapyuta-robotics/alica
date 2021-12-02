#include "OrderedSchedulingTestPlan1629895582410.h"
/*PROTECTED REGION ID(eph1629895582410) ENABLED START*/
// Add additional options here
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
OrderedSchedulingTestPlan1629895582410::OrderedSchedulingTestPlan1629895582410()
        : DomainPlan()
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
/**
 * Transition: FromPlanATo PlanB (1629895758611)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: PlanA --> PlanB
 *
 * Precondition: 1629895758612 (1629895758612)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in PlanA:
 *   - PlanA (1629895837159)
 */
bool PreCondition1629895758612::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1629895758611) ENABLED START*/
    std::cout << "The PreCondition 1629895758612 in Transition 'FromPlanATo PlanB' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Transition: FromPlanBTo PlanA (1629895768181)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: PlanB --> PlanA
 *
 * Precondition: 1629895768182 (1629895768182)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in PlanB:
 *   - PlanB (1629895853508)
 */
bool PreCondition1629895768182::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1629895768181) ENABLED START*/
    std::cout << "The PreCondition 1629895768182 in Transition 'FromPlanBTo PlanA' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1629895582410) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
