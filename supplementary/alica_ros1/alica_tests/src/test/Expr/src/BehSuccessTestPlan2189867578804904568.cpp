#include <alica_tests/BehSuccessTestPlan2189867578804904568.h>
/*PROTECTED REGION ID(eph2189867578804904568) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  BehSuccessTestPlan (2189867578804904568)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 3085248359654706812)
//
// States:
//   - BehSuccessOnInitState (1379727748995057687)
BehSuccessTestPlan2189867578804904568::BehSuccessTestPlan2189867578804904568(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con2189867578804904568) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
BehSuccessTestPlan2189867578804904568::~BehSuccessTestPlan2189867578804904568()
{
    /*PROTECTED REGION ID(dcon2189867578804904568) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 3085248359654706812
 */
std::shared_ptr<UtilityFunction> UtilityFunction2189867578804904568::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(2189867578804904568) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods2189867578804904568) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
