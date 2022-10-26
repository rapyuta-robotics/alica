#include <alica_tests/TestParameterPassing1692837668719979457.h>
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
//   - SecondCall (1529456591400)
//   - FirstCall (1092447442809556626)
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

/*PROTECTED REGION ID(methods1692837668719979457) ENABLED START*/
void TestParameterPassing1692837668719979457::onInit()
{
    LockedBlackboardRW bb(*(getBlackboard()));
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(getWorldModel());
    bb.set<PlanStatus>("targetChildStatus", PlanStatus::Success);
    bb.set<int64_t>("planKey", 1);
    wm->passedParameters["planKey"] = bb.get<int64_t>("planKey");
    bb.set<int64_t>("planOutputKey", 5);
    bb.set<int64_t>("planSecondOutputKey", 7);
    bb.set<int64_t>("planInputKey", 1);
    wm->passedParameters["planInputFromMaster"] = bb.get<int64_t>("planInputFromMaster");
}
/*PROTECTED REGION END*/
} // namespace alica
