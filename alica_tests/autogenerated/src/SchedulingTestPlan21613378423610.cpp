#include "SchedulingTestPlan21613378423610.h"
/*PROTECTED REGION ID(eph1613378423610) ENABLED START*/
// Add additional options here
#include "CounterClass.h"
#include <assert.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:SchedulingTestPlan21613378423610
SchedulingTestPlan21613378423610::SchedulingTestPlan21613378423610()
        : DomainPlan("SchedulingTestPlan21613378423610")
{
    /*PROTECTED REGION ID(con1613378423610) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SchedulingTestPlan21613378423610::~SchedulingTestPlan21613378423610()
{
    /*PROTECTED REGION ID(dcon1613378423610) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
/**
 * Task: SchedulerTestSublanEntrypoint  -> EntryPoint-ID: 1613378624077
 */
std::shared_ptr<UtilityFunction> UtilityFunction1613378423610::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1613378423610) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1613378423610) ENABLED START*/
// Add additional options here
void SchedulingTestPlan21613378423610::init()
{
    std::cerr << "init of SchedulingTestPlan2 - " << CounterClass::called << std::endl;
    assert(CounterClass::called == 1 || CounterClass::called == 2);
    CounterClass::called += 1;
}

void SchedulingTestPlan21613378423610::onTermination()
{
    std::cerr << "onTermination of SchedulingTestPlan2 - " << CounterClass::called << std::endl;
    assert(CounterClass::called == 4 || CounterClass::called == 3);
    CounterClass::called += 1;
}
/*PROTECTED REGION END*/
} // namespace alica
