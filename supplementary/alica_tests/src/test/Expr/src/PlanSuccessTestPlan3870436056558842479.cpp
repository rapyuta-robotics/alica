#include <alica_tests/PlanSuccessTestPlan3870436056558842479.h>
/*PROTECTED REGION ID(eph3870436056558842479) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  PlanSuccessTestPlan (3870436056558842479)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 3664113777031838556)
//
// States:
//   - PlanSuccessOnInitState (2532366895059669785)
PlanSuccessTestPlan3870436056558842479::PlanSuccessTestPlan3870436056558842479(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con3870436056558842479) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
PlanSuccessTestPlan3870436056558842479::~PlanSuccessTestPlan3870436056558842479()
{
    /*PROTECTED REGION ID(dcon3870436056558842479) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 3664113777031838556
 */
std::shared_ptr<UtilityFunction> UtilityFunction3870436056558842479::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(3870436056558842479) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods3870436056558842479) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
