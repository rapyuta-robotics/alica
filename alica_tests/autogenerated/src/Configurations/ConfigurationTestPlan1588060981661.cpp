#include "Configurations/ConfigurationTestPlan1588060981661.h"
/*PROTECTED REGION ID(eph1588060981661) ENABLED START*/
#include <CounterClass.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:ConfigurationTestPlan1588060981661
ConfigurationTestPlan1588060981661::ConfigurationTestPlan1588060981661()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1588060981661) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
ConfigurationTestPlan1588060981661::~ConfigurationTestPlan1588060981661()
{
    /*PROTECTED REGION ID(dcon1588060981661) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1588061024407
 */
std::shared_ptr<UtilityFunction> UtilityFunction1588060981661::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1588060981661) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1588253347213, ConditionString: , Comment: MISSING_COMMENT
 *
 * Abstract plans in current state:
 *   - ReadConfigurationBehaviour (1588061129360)
 *   - ReadConfigurationBehaviour (1588061129360)
 *   - ReadConfigurationPlantype (1588061351007)
 *   - ReadConfigurationPlan (1588061334567)
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1588061024407)
 *
 * States in plan:
 *   - Default Name (1588060991102)
 *   - Default Name (1588253341545)
 *
 * Variables of precondition:
 */
bool PreCondition1588253347213::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1588253347211) ENABLED START*/
    return CounterClass::called == 1;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1588060981661) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
