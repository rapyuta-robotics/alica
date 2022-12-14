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

/*PROTECTED REGION ID(methods673160616613514188) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
