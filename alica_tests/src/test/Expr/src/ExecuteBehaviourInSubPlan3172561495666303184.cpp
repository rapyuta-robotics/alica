#include <alica_tests/ExecuteBehaviourInSubPlan3172561495666303184.h>
/*PROTECTED REGION ID(eph3172561495666303184) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  ExecuteBehaviourInSubPlan (3172561495666303184)
//
// Tasks:
//   - Attack (1222613952469) (Entrypoint: 560547937773733569)
//
// States:
//   - Suc (3575867719445223184)
//   - Start (4459593820134418510)
ExecuteBehaviourInSubPlan3172561495666303184::ExecuteBehaviourInSubPlan3172561495666303184(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con3172561495666303184) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
ExecuteBehaviourInSubPlan3172561495666303184::~ExecuteBehaviourInSubPlan3172561495666303184()
{
    /*PROTECTED REGION ID(dcon3172561495666303184) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: Attack  -> EntryPoint-ID: 560547937773733569
 */

UtilityFunction3172561495666303184::UtilityFunction3172561495666303184()
        : BasicUtilityFunction()
{
}

std::shared_ptr<UtilityFunction> UtilityFunction3172561495666303184::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(3172561495666303184) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods3172561495666303184) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
