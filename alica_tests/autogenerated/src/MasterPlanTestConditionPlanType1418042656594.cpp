#include "MasterPlanTestConditionPlanType1418042656594.h"
/*PROTECTED REGION ID(eph1418042656594) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  MasterPlanTestConditionPlanType (1418042656594)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1418042656596)
//
// States:
//   - Start (1418042656595)
//   - Plantype (1418042674811)
MasterPlanTestConditionPlanType1418042656594::MasterPlanTestConditionPlanType1418042656594(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1418042656594) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
MasterPlanTestConditionPlanType1418042656594::~MasterPlanTestConditionPlanType1418042656594()
{
    /*PROTECTED REGION ID(dcon1418042656594) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1418042656596
 */
std::shared_ptr<UtilityFunction> UtilityFunction1418042656594::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1418042656594) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
/**
 * Transition: MISSING_NAME (1418042682960)
 *   - Comment:
 *   - Source2Dest: Start --> Plantype
 *
 * Precondition: MISSING_NAME (1418042683692)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Start:
 */
bool PreCondition1418042683692::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel& wm)
{
    /*PROTECTED REGION ID(1418042682960) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1418042656594) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
