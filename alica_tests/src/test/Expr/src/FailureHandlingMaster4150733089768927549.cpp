#include <alica_tests/FailureHandlingMaster4150733089768927549.h>
/*PROTECTED REGION ID(eph4150733089768927549) ENABLED START*/
// Add additional options here
#include <alica_tests/TestWorldModel.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  FailureHandlingMaster (4150733089768927549)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 722203880690238135)
//
// States:
//   - FailureHandled (4449850763179483831)
//   - FailurePlan (198406198808981916)
FailureHandlingMaster4150733089768927549::FailureHandlingMaster4150733089768927549(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con4150733089768927549) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
FailureHandlingMaster4150733089768927549::~FailureHandlingMaster4150733089768927549()
{
    /*PROTECTED REGION ID(dcon4150733089768927549) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 722203880690238135
 */
std::shared_ptr<UtilityFunction> UtilityFunction4150733089768927549::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(4150733089768927549) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 3194919312481305139 (3194919312481305139)
 *   - Comment:
 *   - Source2Dest: FailurePlan --> FailureHandled
 *
 * Precondition: 488794245455049811 (488794245455049811)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in FailurePlan:
 *   - FailurePlan (631515556091266493)
 */
bool PreCondition488794245455049811::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(3194919312481305139) ENABLED START*/
    const auto twm = dynamic_cast<const alicaTests::TestWorldModel*>(wm);
    return twm->isTransitionCondition3194919312481305139();
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods4150733089768927549) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
