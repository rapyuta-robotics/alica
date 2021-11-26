#include "TestTracingMasterPlan691392966514374878.h"
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
TestTracingMasterPlan691392966514374878::TestTracingMasterPlan691392966514374878(IAlicaWorldModel* wm)
        : DomainPlan(wm)
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
std::shared_ptr<UtilityFunction> UtilityFunction691392966514374878::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(691392966514374878) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
/**
 * Transition: 3626189722064037094 (3626189722064037094)
 *   - Comment:
 *   - Source2Dest: 2832176823961443072 --> 3077460522716760463
 *
 * Precondition: 1840401110297459509 (1840401110297459509)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in 2832176823961443072:
 *   - TestTracingSubPlan (1482512794732634139)
 */
bool PreCondition1840401110297459509::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(3626189722064037094) ENABLED START*/
    return alicaTests::TestWorldModel::getOne()->isPreCondition1840401110297459509();
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods691392966514374878) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
