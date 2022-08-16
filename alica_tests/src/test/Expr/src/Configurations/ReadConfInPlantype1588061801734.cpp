#include <alica_tests/Configurations/ReadConfInPlantype1588061801734.h>
/*PROTECTED REGION ID(eph1588061801734) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  ReadConfInPlantype (1588061801734)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1588103719479)
//
// States:
//   - Default Name (1588103714226)
//   - ConfA (1588246134801)
//   - ConfB (1588246136647)
ReadConfInPlantype1588061801734::ReadConfInPlantype1588061801734(PlanContext& context)
        : DomainPlan(context)
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

/*PROTECTED REGION ID(methods1588061801734) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
