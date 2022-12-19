#include <alica_tests/ParallelSuccessOnCondPlan3288843407985944525.h>
/*PROTECTED REGION ID(eph3288843407985944525) ENABLED START*/
// Add additional options here
#include <alica_tests/TestWorldModel.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  ParallelSuccessOnCondPlan (3288843407985944525)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1461040637399587631)
//
// States:
//   - ParallelPlanSuccessState (4136091023931334823)
//   - WaitForTriggerState (1899337663064771436)
//   - ParallelExecState (1413250009777584468)
ParallelSuccessOnCondPlan3288843407985944525::ParallelSuccessOnCondPlan3288843407985944525(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con3288843407985944525) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
ParallelSuccessOnCondPlan3288843407985944525::~ParallelSuccessOnCondPlan3288843407985944525()
{
    /*PROTECTED REGION ID(dcon3288843407985944525) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1461040637399587631
 */
std::shared_ptr<UtilityFunction> UtilityFunction3288843407985944525::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(3288843407985944525) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 4409408749212342883 (4409408749212342883)
 *   - Comment:
 *   - Source2Dest: WaitForTriggerState --> ParallelExecState
 *
 * Precondition: 1470823850869867131 (1470823850869867131)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in WaitForTriggerState:
 */
bool PreCondition1470823850869867131::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(4409408749212342883) ENABLED START*/
    std::cout << "The PreCondition 1470823850869867131 in Transition '4409408749212342883' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 602472383731208509 (602472383731208509)
 *   - Comment:
 *   - Source2Dest: ParallelExecState --> ParallelPlanSuccessState
 *
 * Precondition: 2767999024419231358 (2767999024419231358)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in ParallelExecState:
 *   - SuccessOnCondWrapperA (673160616613514188)
 *   - SuccessOnCondWrapperB (2869465844414224272)
 */
bool PreCondition2767999024419231358::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(602472383731208509) ENABLED START*/
    std::cout << "The PreCondition 2767999024419231358 in Transition '602472383731208509' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods3288843407985944525) ENABLED START*/
// Add additional options here
void ParallelSuccessOnCondPlan3288843407985944525::onInit()
{
    auto* wm = dynamic_cast<alicaTests::TestWorldModelNew*>(getWorldModel());
    auto* tc = wm->getTestContext();
    tc->resetAllTransitions(getName());
}
/*PROTECTED REGION END*/
} // namespace alica
