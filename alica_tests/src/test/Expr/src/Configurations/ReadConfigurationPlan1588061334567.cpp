#include <alica_tests/Configurations/ReadConfigurationPlan1588061334567.h>
/*PROTECTED REGION ID(eph1588061334567) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  ReadConfigurationPlan (1588061334567)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1588069183324)
//
// States:
//   - DecisionState (1588069177860)
//   - StateA (1588069261047)
//   - StateB (1588069265377)
ReadConfigurationPlan1588061334567::ReadConfigurationPlan1588061334567(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1588061334567) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
ReadConfigurationPlan1588061334567::~ReadConfigurationPlan1588061334567()
{
    /*PROTECTED REGION ID(dcon1588061334567) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1588069183324
 */

UtilityFunction1588061334567::UtilityFunction1588061334567()
        : BasicUtilityFunction()
{
}

std::shared_ptr<UtilityFunction> UtilityFunction1588061334567::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1588061334567) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1588061334567) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
