#include "TestParameterPassingMaster1179066429431332055.h"
/*PROTECTED REGION ID(eph1179066429431332055) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  TestParameterPassingMaster (1179066429431332055)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 4058387577648167302)
//
// States:
//   - ParameterPassingRunSubPlan (2069338196796962570)
TestParameterPassingMaster1179066429431332055::TestParameterPassingMaster1179066429431332055(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1179066429431332055) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
TestParameterPassingMaster1179066429431332055::~TestParameterPassingMaster1179066429431332055()
{
    /*PROTECTED REGION ID(dcon1179066429431332055) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 4058387577648167302
 */
std::shared_ptr<UtilityFunction> UtilityFunction1179066429431332055::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1179066429431332055) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Set parameters for child abstract plan TestParameterPassing1692837668719979457 of state ParameterPassingRunSubPlan2069338196796962570
 */
bool PlanAttachment105160539449888459::setParameters(const Blackboard& parent_bb, Blackboard& child_bb)
{
    /*PROTECTED REGION ID(105160539449888459) ENABLED START*/
    LockedBlackboardRW bb = LockedBlackboardRW(child_bb);
    bb.registerValue("behaviourParameter", 1);
    bb.registerValue("planParameter", 2);
    return true;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1179066429431332055) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica