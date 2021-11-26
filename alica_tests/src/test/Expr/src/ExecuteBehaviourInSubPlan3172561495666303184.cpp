#include "ExecuteBehaviourInSubPlan3172561495666303184.h"
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
ExecuteBehaviourInSubPlan3172561495666303184::ExecuteBehaviourInSubPlan3172561495666303184(IAlicaWorldModel* wm)
        : DomainPlan(wm)
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
std::shared_ptr<UtilityFunction> UtilityFunction3172561495666303184::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(3172561495666303184) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
/**
 * Transition: 2330492839242485043 (2330492839242485043)
 *   - Comment:
 *   - Source2Dest: Start --> Suc
 *
 * Precondition: ToSuc (1943478533524176732)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Start:
 *   - TestBehaviour (55178365253414982)
 */
bool PreCondition1943478533524176732::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(2330492839242485043) ENABLED START*/
    std::cout << "The PreCondition 1943478533524176732 in Transition '2330492839242485043' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods3172561495666303184) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
