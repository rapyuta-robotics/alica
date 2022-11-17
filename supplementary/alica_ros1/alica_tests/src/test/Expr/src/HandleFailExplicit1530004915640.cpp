#include <alica_tests/HandleFailExplicit1530004915640.h>
/*PROTECTED REGION ID(eph1530004915640) ENABLED START*/
// Add additional using directives here
#include <alica_tests/SimpleSwitches.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  HandleFailExplicit (1530004915640)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1530004915642)
//
// States:
//   - A (1530004915641)
//   - B (1530004973591)
//   - C (1530004975275)
//   - D (1532424087894)
//   - E (1532424097662)
HandleFailExplicit1530004915640::HandleFailExplicit1530004915640(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1530004915640) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
HandleFailExplicit1530004915640::~HandleFailExplicit1530004915640()
{
    /*PROTECTED REGION ID(dcon1530004915640) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1530004915642
 */
std::shared_ptr<UtilityFunction> UtilityFunction1530004915640::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1530004915640) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1530004915640) ENABLED START*/
// Add additional options here
void HandleFailExplicit1530004915640::onInit()
{
    LockedBlackboardRW bb(*(getBlackboard()));
    bb.set<int64_t>("aToBSwitch", 0);
    bb.set<int64_t>("cToDSwitch", 2);
}
/*PROTECTED REGION END*/
} // namespace alica
