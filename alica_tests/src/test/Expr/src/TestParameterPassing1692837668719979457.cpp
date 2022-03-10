#include "TestParameterPassing1692837668719979457.h"
/*PROTECTED REGION ID(eph1692837668719979457) ENABLED START*/
// Add additional options here
#include "alica_tests/TestWorldModel.h"
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  TestParameterPassing (1692837668719979457)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 58084702421574748)
//
// States:
//   - FirstCall (1092447442809556626)
//   - SecondCall (1529456591400)
TestParameterPassing1692837668719979457::TestParameterPassing1692837668719979457(PlanContext& context)
        : DomainPlan(context)
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
 * Transition: MISSING_NAME (1129456609900)
 *   - Comment: Forth
 *   - Source2Dest: FirstCall --> SecondCall
 *
 * Precondition: MISSING_NAME (1529456610600)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in FirstCall:
 *   - TestParameterPassingBehaviour (831400441334251602)
 */
bool PreCondition1529456610600::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1129456609900) ENABLED START*/
    return rp->isAnyChildStatus(PlanStatus::Success);
    /*PROTECTED REGION END*/
}

/**
 * Transition: MISSING_NAME (2229456609900)
 *   - Comment: Back
 *   - Source2Dest: SecondCall --> FirstCall
 *
 * Precondition: MISSING_NAME (2529456610600)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in SecondCall:
 *   - TestParameterPassingBehaviour (831400441334251602)
 */
bool PreCondition2529456610600::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(2229456609900) ENABLED START*/
    LockedBlackboardRO bb = LockedBlackboardRO(*(rp->getBasicPlan()->getBlackboard()));
    auto testWm = const_cast<alicaTests::TestWorldModel*>(dynamic_cast<const alicaTests::TestWorldModel*>(wm));
    testWm->passedParameters["planInputKey"] = bb.get<int>("planInputKey");
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1692837668719979457) ENABLED START*/
void TestParameterPassing1692837668719979457::onInit()
{
    LockedBlackboardRW bb = LockedBlackboardRW(*(getBlackboard()));
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(getWorldModel());
    bb.set("planKey", 1);
    wm->passedParameters["planKey"] = bb.get<int>("planKey");
    bb.set("planOutputKey", 5);
    bb.set("planSecondOutputKey", 7);
    bb.set("planInputKey", 1);
    wm->passedParameters["planInputFromMaster"] = bb.get<int>("planInputFromMaster");
}
/*PROTECTED REGION END*/
} // namespace alica
