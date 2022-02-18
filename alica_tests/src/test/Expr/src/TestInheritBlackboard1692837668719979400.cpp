#include "TestInheritBlackboard1692837668719979400.h"
/*PROTECTED REGION ID(eph1692837668719979400) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  TestInheritBlackboard (1692837668719979400)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 58084702421574700)
//
// States:
//   - InheritBlackboardRunBehaviour (1092447442809556600)
TestInheritBlackboard1692837668719979400::TestInheritBlackboard1692837668719979400(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1692837668719979400) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
TestInheritBlackboard1692837668719979400::~TestInheritBlackboard1692837668719979400()
{
    /*PROTECTED REGION ID(dcon1692837668719979400) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 58084702421574700
 */
std::shared_ptr<UtilityFunction> UtilityFunction1692837668719979400::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1692837668719979400) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1692837668719979400) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
