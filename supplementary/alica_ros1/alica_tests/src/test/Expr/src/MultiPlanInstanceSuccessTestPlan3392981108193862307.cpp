#include <alica_tests/MultiPlanInstanceSuccessTestPlan3392981108193862307.h>
/*PROTECTED REGION ID(eph3392981108193862307) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  MultiPlanInstanceSuccessTestPlan (3392981108193862307)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 3479279297067523171)
//
// States:
//   - ParallelSuccessOnCondState (171053979538031088)
MultiPlanInstanceSuccessTestPlan3392981108193862307::MultiPlanInstanceSuccessTestPlan3392981108193862307(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con3392981108193862307) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
MultiPlanInstanceSuccessTestPlan3392981108193862307::~MultiPlanInstanceSuccessTestPlan3392981108193862307()
{
    /*PROTECTED REGION ID(dcon3392981108193862307) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 3479279297067523171
 */
std::shared_ptr<UtilityFunction> UtilityFunction3392981108193862307::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(3392981108193862307) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods3392981108193862307) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
