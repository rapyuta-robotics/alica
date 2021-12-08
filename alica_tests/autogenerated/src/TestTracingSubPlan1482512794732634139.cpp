#include "TestTracingSubPlan1482512794732634139.h"
/*PROTECTED REGION ID(eph1482512794732634139) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  TestTracingSubPlan (1482512794732634139)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1733191353578191428)
//
// States:
//   - 3860426216975738 (3860426216975738)
TestTracingSubPlan1482512794732634139::TestTracingSubPlan1482512794732634139(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1482512794732634139) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
TestTracingSubPlan1482512794732634139::~TestTracingSubPlan1482512794732634139()
{
    /*PROTECTED REGION ID(dcon1482512794732634139) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1733191353578191428
 */
std::shared_ptr<UtilityFunction> UtilityFunction1482512794732634139::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1482512794732634139) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1482512794732634139) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
