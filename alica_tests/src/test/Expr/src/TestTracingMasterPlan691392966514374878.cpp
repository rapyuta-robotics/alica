#include <alica_tests/TestTracingMasterPlan691392966514374878.h>
/*PROTECTED REGION ID(eph691392966514374878) ENABLED START*/
// Add additional options here
#include <alica_tests/TestWorldModel.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  TestTracingMasterPlan (691392966514374878)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 4059536593953041663)
//
// States:
//   - 2832176823961443072 (2832176823961443072)
//   - 3077460522716760463 (3077460522716760463)
TestTracingMasterPlan691392966514374878::TestTracingMasterPlan691392966514374878(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con691392966514374878) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
TestTracingMasterPlan691392966514374878::~TestTracingMasterPlan691392966514374878()
{
    /*PROTECTED REGION ID(dcon691392966514374878) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 4059536593953041663
 */

UtilityFunction691392966514374878::UtilityFunction691392966514374878()
        : BasicUtilityFunction()
{
}

std::shared_ptr<UtilityFunction> UtilityFunction691392966514374878::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(691392966514374878) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods691392966514374878) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
