#include "DynamicTaskAssignmentTestMaster1602078208698393838.h"
/*PROTECTED REGION ID(eph1602078208698393838) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  DynamicTaskAssignmentTestMaster (1602078208698393838)
//
// Tasks:
//   - DynamicTaskTestEP (3903894018484081749) (Entrypoint: 699381937789438517)
//
// States:
//   - Init (4467904887554008050)
//   - Finish (1317277234576050904)
//   - Start (751302000461175045)
DynamicTaskAssignmentTestMaster1602078208698393838::DynamicTaskAssignmentTestMaster1602078208698393838()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1602078208698393838) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
DynamicTaskAssignmentTestMaster1602078208698393838::~DynamicTaskAssignmentTestMaster1602078208698393838()
{
    /*PROTECTED REGION ID(dcon1602078208698393838) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DynamicTaskTestEP  -> EntryPoint-ID: 699381937789438517
 */
std::shared_ptr<UtilityFunction> UtilityFunction1602078208698393838::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1602078208698393838) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
/**
 * Transition: 4266666033623620026 (4266666033623620026)
 *   - Comment:
 *   - Source2Dest: Init --> Start
 *
 * Precondition: InitDone (4496654201854254411)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Init:
 */
bool PreCondition4496654201854254411::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(4266666033623620026) ENABLED START*/
    std::cout << "The PreCondition 4496654201854254411 in Transition '4266666033623620026' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Transition: 3712615202042019043 (3712615202042019043)
 *   - Comment:
 *   - Source2Dest: Start --> Finish
 *
 * Precondition: 4344644064496100420 (4344644064496100420)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Start:
 *   - DynamicTaskAssignmentTest (2252865124432942907)
 */
bool PreCondition4344644064496100420::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(3712615202042019043) ENABLED START*/
    std::cout << "The PreCondition 4344644064496100420 in Transition '3712615202042019043' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1602078208698393838) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
