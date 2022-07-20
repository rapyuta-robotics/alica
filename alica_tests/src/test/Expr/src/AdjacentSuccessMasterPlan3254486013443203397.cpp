#include <alica_tests/AdjacentSuccessMasterPlan3254486013443203397.h>
/*PROTECTED REGION ID(eph3254486013443203397) ENABLED START*/
// Add additional options here
#include <alica_tests/TestWorldModel.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  AdjacentSuccessMasterPlan (3254486013443203397)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 177437342277134781)
//
// States:
//   - EntryState (338845808462999166)
//   - SecondState (1114306208475690481)
AdjacentSuccessMasterPlan3254486013443203397::AdjacentSuccessMasterPlan3254486013443203397(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con3254486013443203397) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
AdjacentSuccessMasterPlan3254486013443203397::~AdjacentSuccessMasterPlan3254486013443203397()
{
    /*PROTECTED REGION ID(dcon3254486013443203397) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 177437342277134781
 */
std::shared_ptr<UtilityFunction> UtilityFunction3254486013443203397::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(3254486013443203397) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods3254486013443203397) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
