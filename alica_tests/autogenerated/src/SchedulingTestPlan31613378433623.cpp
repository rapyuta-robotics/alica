#include "SchedulingTestPlan31613378433623.h"
/*PROTECTED REGION ID(eph1613378433623) ENABLED START*/
// Add additional options here
#include "CounterClass.h"
#include <assert.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:SchedulingTestPlan31613378433623
SchedulingTestPlan31613378433623::SchedulingTestPlan31613378433623()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1613378433623) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SchedulingTestPlan31613378433623::~SchedulingTestPlan31613378433623()
{
    /*PROTECTED REGION ID(dcon1613378433623) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
/**
 * Task: SchedulerTestSubPlanEntrypoint  -> EntryPoint-ID: 1613378578364
 */
std::shared_ptr<UtilityFunction> UtilityFunction1613378433623::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1613378433623) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1613378433623) ENABLED START*/
// Add additional options here
void SchedulingTestPlan31613378433623::init()
{
    CounterClass::called += 1;
}

void SchedulingTestPlan31613378433623::onTermination()
{
    CounterClass::called += 1;
}
/*PROTECTED REGION END*/
} // namespace alica
