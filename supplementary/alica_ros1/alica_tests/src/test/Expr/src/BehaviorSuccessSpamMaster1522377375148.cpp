#include <alica_tests/BehaviorSuccessSpamMaster1522377375148.h>
/*PROTECTED REGION ID(eph1522377375148) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  BehaviorSuccessSpamMaster (1522377375148)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1522377375150)
//
// States:
//   - Normal (1522377375149)
//   - Dummy (1522377929290)
BehaviorSuccessSpamMaster1522377375148::BehaviorSuccessSpamMaster1522377375148(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1522377375148) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
BehaviorSuccessSpamMaster1522377375148::~BehaviorSuccessSpamMaster1522377375148()
{
    /*PROTECTED REGION ID(dcon1522377375148) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1522377375150
 */
std::shared_ptr<UtilityFunction> UtilityFunction1522377375148::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1522377375148) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1522377375148) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
