#include "SchedulingTestPlan11613378406860.h"
/*PROTECTED REGION ID(eph1613378406860) ENABLED START*/
// Add additional options here
#include "CounterClass.h"
#include <assert.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:SchedulingTestPlan11613378406860
SchedulingTestPlan11613378406860::SchedulingTestPlan11613378406860()
        : DomainPlan("SchedulingTestPlan11613378406860")
{
    /*PROTECTED REGION ID(con1613378406860) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SchedulingTestPlan11613378406860::~SchedulingTestPlan11613378406860()
{
    /*PROTECTED REGION ID(dcon1613378406860) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
/**
 * Task: SchedulerTestSublanEntrypoint  -> EntryPoint-ID: 1613378541158
 */
std::shared_ptr<UtilityFunction> UtilityFunction1613378406860::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1613378406860) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1613977426634, ConditionString: , Comment: MISSING_COMMENT
 *
 * Abstract plans in current state:
 *   - SchedulingTestPlan3 (1613378433623)
 *   - SchedulingTestPlan2 (1613378423610)
 *
 * Tasks in plan:
 *   - SchedulerTestSublanEntrypoint (1613372009777) (Entrypoint: 1613378541158)
 *
 * States in plan:
 *   - Default Name (1613378543512)
 *   - Default Name (1613977406218)
 *
 * Variables of precondition:
 */
bool PreCondition1613977426634::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1613977426633) ENABLED START*/
    std::cerr << "The PreCondition 1613977426634 in Transition 'FromDefault NameTo Default Name' is not implement yet!" << std::endl;
    return true;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1613378406860) ENABLED START*/
// Add additional options here
void SchedulingTestPlan11613378406860::init()
{
    std::cerr << "init of SchedulingTestPlan1" << std::endl;
    CounterClass::called = 1;
}

void SchedulingTestPlan11613378406860::onTermination()
{
    assert(CounterClass::called == 2);
    CounterClass::called = 0;
    std::cerr << "onTermination of SchedulingTestPlan1" << std::endl;
}
/*PROTECTED REGION END*/
} // namespace alica
