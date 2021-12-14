#include "TestParameterPassing1692837668719979457.h"
/*PROTECTED REGION ID(eph1692837668719979457) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  TestParameterPassing (1692837668719979457)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 58084702421574748)
//
// States:
//   - ParameterPassingRunBehaviour (1092447442809556626)
TestParameterPassing1692837668719979457::TestParameterPassing1692837668719979457(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1692837668719979457) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
TestParameterPassing1692837668719979457::~TestParameterPassing1692837668719979457()
{
    /*PROTECTED REGION ID(dcon1692837668719979457) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 58084702421574748
 */
std::shared_ptr<UtilityFunction> UtilityFunction1692837668719979457::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1692837668719979457) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Set parameters for child abstract plan TestParameterPassingBehaviour831400441334251602 of state ParameterPassingRunBehaviour1092447442809556626
 */
bool PlanAttachment445396005944825225::setParameters(const Blackboard& parent_bb, Blackboard& child_bb)
{
    /*PROTECTED REGION ID(445396005944825225) ENABLED START*/
    std::cerr << "Calling setParameter in SubPlan!" << std::endl;
    LockedBlackboardRO parent = LockedBlackboardRO(parent_bb);
    LockedBlackboardRW child = LockedBlackboardRW(child_bb);
    child.registerValue("behaviourParameter", parent.get<int>("behaviourParameter"));
    std::cerr << "registered parameters, returning" << std::endl;
    return true;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1692837668719979457) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
