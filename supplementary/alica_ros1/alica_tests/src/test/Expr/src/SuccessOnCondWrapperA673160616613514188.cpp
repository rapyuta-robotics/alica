#include <alica_tests/SuccessOnCondWrapperA673160616613514188.h>
/*PROTECTED REGION ID(eph673160616613514188) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  SuccessOnCondWrapperA (673160616613514188)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1329563598549023051)
//
// States:
//   - SuccessOnCondState (2511956906797886911)
//   - WrappeASuccessState (309824621058621265)
SuccessOnCondWrapperA673160616613514188::SuccessOnCondWrapperA673160616613514188(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con673160616613514188) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SuccessOnCondWrapperA673160616613514188::~SuccessOnCondWrapperA673160616613514188()
{
    /*PROTECTED REGION ID(dcon673160616613514188) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1329563598549023051
 */
std::shared_ptr<UtilityFunction> UtilityFunction673160616613514188::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(673160616613514188) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 1123795990168202938 (1123795990168202938)
 *   - Comment:
 *   - Source2Dest: SuccessOnCondState --> WrappeASuccessState
 *
 * Precondition: 4605367163774150375 (4605367163774150375)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in SuccessOnCondState:
 *   - SuccessOnCondPlan (3153116020668535682)
 */
bool PreCondition4605367163774150375::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1123795990168202938) ENABLED START*/
    std::cout << "The PreCondition 4605367163774150375 in Transition '1123795990168202938' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods673160616613514188) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
