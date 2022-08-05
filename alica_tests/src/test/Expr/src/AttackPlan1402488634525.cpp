#include <alica_tests/AttackPlan1402488634525.h>
/*PROTECTED REGION ID(eph1402488634525) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  AttackPlan (1402488634525)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1402488646221)
//
// States:
//   - Attack (1402488646220)
//   - Shoot (1402489396914)
AttackPlan1402488634525::AttackPlan1402488634525(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1402488634525) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
AttackPlan1402488634525::~AttackPlan1402488634525()
{
    /*PROTECTED REGION ID(dcon1402488634525) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1402488646221
 */

UtilityFunction1402488634525::UtilityFunction1402488634525()
        : BasicUtilityFunction()
{
}

std::shared_ptr<UtilityFunction> UtilityFunction1402488634525::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1402488634525) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1402488634525) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
