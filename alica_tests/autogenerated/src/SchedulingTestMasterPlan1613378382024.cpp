#include "SchedulingTestMasterPlan1613378382024.h"
/*PROTECTED REGION ID(eph1613378382024) ENABLED START*/
// Add additional options here
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
SchedulingTestMasterPlan1613378382024::SchedulingTestMasterPlan1613378382024()
        : DomainPlan()
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
bool PreCondition1613530643882::evaluate(std::shared_ptr<RunningPlan> rp)
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
bool PreCondition1615797316171::evaluate(std::shared_ptr<RunningPlan> rp)
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
bool PreCondition1615797327077::evaluate(std::shared_ptr<RunningPlan> rp)
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
bool PreCondition1629895598471::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1629895598464) ENABLED START*/
    std::cout << "The PreCondition 1629895598471 in Transition 'FromDefault NameTo Default Name' is not implement yet!" << std::endl;
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
bool PreCondition1615797365364::evaluate(std::shared_ptr<RunningPlan> rp)
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
bool PreCondition1629895607018::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1629895607017) ENABLED START*/
    std::cout << "The PreCondition 1629895607018 in Transition 'FromDefault NameTo EndTest' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1613378382024) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
