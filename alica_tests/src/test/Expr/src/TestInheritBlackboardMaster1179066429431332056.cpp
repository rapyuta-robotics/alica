#include <alica_tests/TestInheritBlackboardMaster1179066429431332056.h>
/*PROTECTED REGION ID(eph1179066429431332056) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  TestInheritBlackboardMaster (1179066429431332056)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 4058387577648167303)
//
// States:
//   - InheritBlackboardRunSubPlan (2069338196796962571)
TestInheritBlackboardMaster1179066429431332056::TestInheritBlackboardMaster1179066429431332056(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1179066429431332056) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
TestInheritBlackboardMaster1179066429431332056::~TestInheritBlackboardMaster1179066429431332056()
{
    /*PROTECTED REGION ID(dcon1179066429431332056) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 4058387577648167303
 */
std::shared_ptr<UtilityFunction> UtilityFunction1179066429431332056::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1179066429431332056) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1179066429431332056) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
