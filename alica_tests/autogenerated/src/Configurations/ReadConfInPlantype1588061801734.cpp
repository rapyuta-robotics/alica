#include "Configurations/ReadConfInPlantype1588061801734.h"
/*PROTECTED REGION ID(eph1588061801734) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:ReadConfInPlantype1588061801734
ReadConfInPlantype1588061801734::ReadConfInPlantype1588061801734()
        : DomainPlan("ReadConfInPlantype1588061801734")
{
    /*PROTECTED REGION ID(con1588061801734) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
ReadConfInPlantype1588061801734::~ReadConfInPlantype1588061801734()
{
    /*PROTECTED REGION ID(dcon1588061801734) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1588103719479
 */
std::shared_ptr<UtilityFunction> UtilityFunction1588061801734::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1588061801734) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1588246144841, ConditionString: , Comment: MISSING_COMMENT
 *
 * Abstract plans in current state:
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1588103719479)
 *
 * States in plan:
 *   - Default Name (1588103714226)
 *   - ConfA (1588246134801)
 *   - ConfB (1588246136647)
 *
 * Variables of precondition:
 */
bool PreCondition1588246144841::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1588246144840) ENABLED START*/
    std::string value;
    rp->getParameter("TestValue", value);
    return value.compare("2") == 0;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1588246141557, ConditionString: , Comment: MISSING_COMMENT
 *
 * Abstract plans in current state:
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1588103719479)
 *
 * States in plan:
 *   - Default Name (1588103714226)
 *   - ConfA (1588246134801)
 *   - ConfB (1588246136647)
 *
 * Variables of precondition:
 */
bool PreCondition1588246141557::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1588246141555) ENABLED START*/
    std::string value;
    rp->getParameter("TestValue", value);
    return value.compare("1") == 0;
    /*PROTECTED REGION END*/
}
} // namespace alica
