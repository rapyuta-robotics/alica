#include "SchedulingTestSequenceSubPlan31614964478264.h"
/*PROTECTED REGION ID(eph1614964478264) ENABLED START*/
// Add additional options here
#include "engine/RunningPlan.h"
#include <alica_tests/TestWorldModel.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  SchedulingTestSequenceSubPlan3 (1614964478264)
//
// Tasks:
//   - SchedulerTestSubPlanEntrypoint (1613372009777) (Entrypoint: 1614964917149)
//
// States:
//   - InitSequenceSubPlan3 (1614964919973)
SchedulingTestSequenceSubPlan31614964478264::SchedulingTestSequenceSubPlan31614964478264(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1614964478264) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SchedulingTestSequenceSubPlan31614964478264::~SchedulingTestSequenceSubPlan31614964478264()
{
    /*PROTECTED REGION ID(dcon1614964478264) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: SchedulerTestSubPlanEntrypoint  -> EntryPoint-ID: 1614964917149
 */
std::shared_ptr<UtilityFunction> UtilityFunction1614964478264::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1614964478264) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1614964478264) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
