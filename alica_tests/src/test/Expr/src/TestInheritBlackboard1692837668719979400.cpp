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

/**
 * Set parameters for child abstract plan TestInheritBlackboardBehaviour831400441334251600 of state InheritBlackboardRunBehaviour1092447442809556600
 */
bool PlanAttachment445396005944825200::setParameters(const Blackboard& parent_bb, Blackboard& child_bb)
{
    /*PROTECTED REGION ID(445396005944825200) ENABLED START*/
    std::cout << "The Parameter setter 445396005944825200 in is not implemented yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1692837668719979400) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
