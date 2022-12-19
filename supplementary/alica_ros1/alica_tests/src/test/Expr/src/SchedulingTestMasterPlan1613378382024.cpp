#include <alica_tests/SchedulingTestMasterPlan1613378382024.h>
/*PROTECTED REGION ID(eph1613378382024) ENABLED START*/
// Add additional options here
#include <alica_tests/test_sched_world_model.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  SchedulingTestMasterPlan (1613378382024)
//
// Tasks:
//   - SchedulerTestEntrypoint (1613371619454) (Entrypoint: 1615797283419)
//
// States:
//   - InitTest (1613378474109)
//   - EndTest (1613530614559)
//   - Default Name (1615797271229)
//   - Default Name (1615797319003)
//   - OrderedSchedulingTestPlan (1629895593451)
//   - ExecuteBehaviour (1206766322278521913)
//   - ExecuteBehaviourInSubPlan (3802371674214346622)
SchedulingTestMasterPlan1613378382024::SchedulingTestMasterPlan1613378382024(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1613378382024) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SchedulingTestMasterPlan1613378382024::~SchedulingTestMasterPlan1613378382024()
{
    /*PROTECTED REGION ID(dcon1613378382024) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: SchedulerTestEntrypoint  -> EntryPoint-ID: 1615797283419
 */
std::shared_ptr<UtilityFunction> UtilityFunction1613378382024::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1613378382024) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Transition: FromDefault NameTo EndTest (1613530643879)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: InitTest --> EndTest
 *
 * Precondition: 1613530643882 (1613530643882)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in InitTest:
 *   - SchedulingTestPlan1 (1613378406860)
 */
bool PreCondition1613530643882::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1613530643879) ENABLED START*/
    std::cout << "The PreCondition 1613530643882 in Transition 'FromDefault NameTo EndTest' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: FromDefault NameTo InitTest (1615797316170)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: Default Name --> InitTest
 *
 * Precondition: 1615797316171 (1615797316171)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Default Name:
 */
bool PreCondition1615797316171::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1615797316170) ENABLED START*/
    std::cout << "The PreCondition 1615797316171 in Transition 'FromDefault NameTo InitTest' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: FromDefault NameTo Default Name (1615797327076)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: Default Name --> Default Name
 *
 * Precondition: 1615797327077 (1615797327077)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Default Name:
 */
bool PreCondition1615797327077::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1615797327076) ENABLED START*/
    std::cout << "The PreCondition 1615797327077 in Transition 'FromDefault NameTo Default Name' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: FromDefault NameTo Default Name (1629895598464)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: Default Name --> OrderedSchedulingTestPlan
 *
 * Precondition: 1629895598471 (1629895598471)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Default Name:
 */
bool PreCondition1629895598471::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1629895598464) ENABLED START*/
    std::cout << "The PreCondition 1629895598471 in Transition 'FromDefault NameTo Default Name' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 3351673290341906102 (3351673290341906102)
 *   - Comment:
 *   - Source2Dest: Default Name --> ExecuteBehaviour
 *
 * Precondition: ToExecutePlan (61978004585920576)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Default Name:
 */
bool PreCondition61978004585920576::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(3351673290341906102) ENABLED START*/
    std::cout << "The PreCondition 61978004585920576 in Transition '3351673290341906102' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: FromDefault NameTo EndTest (1615797365363)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: Default Name --> EndTest
 *
 * Precondition: 1615797365364 (1615797365364)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Default Name:
 *   - SchedulingTestSequencePlan1 (1614963946725)
 */
bool PreCondition1615797365364::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1615797365363) ENABLED START*/
    std::cout << "The PreCondition 1615797365364 in Transition 'FromDefault NameTo EndTest' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: FromDefault NameTo EndTest (1629895607017)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: OrderedSchedulingTestPlan --> EndTest
 *
 * Precondition: 1629895607018 (1629895607018)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in OrderedSchedulingTestPlan:
 *   - OrderedSchedulingTestPlan (1629895582410)
 */
bool PreCondition1629895607018::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1629895607017) ENABLED START*/
    std::cout << "The PreCondition 1629895607018 in Transition 'FromDefault NameTo EndTest' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: ExecuteBehaviourToExecuteBehaviourInSubPlan (383854659955639601)
 *   - Comment:
 *   - Source2Dest: ExecuteBehaviour --> ExecuteBehaviourInSubPlan
 *
 * Precondition: ToExecutePlanAsSubPlan (3213510506830850443)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in ExecuteBehaviour:
 *   - TestBehaviour (55178365253414982)
 */
bool PreCondition3213510506830850443::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(383854659955639601) ENABLED START*/
    std::cout << "The PreCondition 3213510506830850443 in Transition 'ExecuteBehaviourToExecuteBehaviourInSubPlan' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 1506708037135242126 (1506708037135242126)
 *   - Comment:
 *   - Source2Dest: ExecuteBehaviourInSubPlan --> EndTest
 *
 * Precondition: ToEndTest (68542020926196536)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in ExecuteBehaviourInSubPlan:
 *   - ExecuteBehaviourInSubPlan (3172561495666303184)
 */
bool PreCondition68542020926196536::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1506708037135242126) ENABLED START*/
    std::cout << "The PreCondition 68542020926196536 in Transition '1506708037135242126' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: ExecuteBehaviourInSubPlanToExecuteBehaviour (1773144683253207826)
 *   - Comment:
 *   - Source2Dest: ExecuteBehaviourInSubPlan --> ExecuteBehaviour
 *
 * Precondition: 4165333637052704488 (4165333637052704488)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in ExecuteBehaviourInSubPlan:
 *   - ExecuteBehaviourInSubPlan (3172561495666303184)
 */
bool PreCondition4165333637052704488::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1773144683253207826) ENABLED START*/
    std::cout << "The PreCondition 4165333637052704488 in Transition 'ExecuteBehaviourInSubPlanToExecuteBehaviour' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1613378382024) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
