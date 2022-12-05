#include <alica_tests/TestMasterPlan2521443078354411465.h>
/*PROTECTED REGION ID(eph2521443078354411465) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  TestMasterPlan (2521443078354411465)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 3091576485060406140)
//
// States:
//   - PlanSuccessTestState (2212831089687963769)
//   - ChooseTestState (4098979167613947533)
//   - BehSuccessTestState (4487929496627066142)
TestMasterPlan2521443078354411465::TestMasterPlan2521443078354411465(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con2521443078354411465) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
TestMasterPlan2521443078354411465::~TestMasterPlan2521443078354411465()
{
    /*PROTECTED REGION ID(dcon2521443078354411465) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 3091576485060406140
 */
std::shared_ptr<UtilityFunction> UtilityFunction2521443078354411465::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(2521443078354411465) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods2521443078354411465) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
