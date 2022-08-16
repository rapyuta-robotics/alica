#include <alica_tests/FailureHandlingMaster4150733089768927549.h>
/*PROTECTED REGION ID(eph4150733089768927549) ENABLED START*/
// Add additional options here
#include <alica_tests/TestWorldModel.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  FailureHandlingMaster (4150733089768927549)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 722203880690238135)
//
// States:
//   - FailurePlan (198406198808981916)
//   - FailureHandled (4449850763179483831)
FailureHandlingMaster4150733089768927549::FailureHandlingMaster4150733089768927549(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con4150733089768927549) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
FailureHandlingMaster4150733089768927549::~FailureHandlingMaster4150733089768927549()
{
    /*PROTECTED REGION ID(dcon4150733089768927549) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 722203880690238135
 */
std::shared_ptr<UtilityFunction> UtilityFunction4150733089768927549::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(4150733089768927549) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods4150733089768927549) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
