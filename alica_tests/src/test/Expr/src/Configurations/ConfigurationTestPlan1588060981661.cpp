#include <alica_tests/Configurations/ConfigurationTestPlan1588060981661.h>
/*PROTECTED REGION ID(eph1588060981661) ENABLED START*/
#include <alica_tests/CounterClass.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  ConfigurationTestPlan (1588060981661)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1588061024407)
//
// States:
//   - Default Name (1588060991102)
//   - Default Name (1588253341545)
ConfigurationTestPlan1588060981661::ConfigurationTestPlan1588060981661(PlanContext& context)
        : DomainPlan(context)
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

/*PROTECTED REGION ID(methods1588060981661) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
