#include <alica_tests/TestMasterPlan2521443078354411465.h>
/*PROTECTED REGION ID(eph2521443078354411465) ENABLED START*/
// Add additional options here
#include <alica_tests/TestWorldModel.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  TestMasterPlan (2521443078354411465)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 3091576485060406140)
//
// States:
//   - SimpleTestPlanState (1506398272124884391)
//   - PlanIsSuccess (1552041115313373349)
//   - PlanSuccessTestState (2212831089687963769)
//   - RunBehaviourInSimplePlanState (3760344851508420724)
//   - MultiPlanInstanceSuccessTestState (3960396736820956915)
//   - RuntimeConditionCalledPlanState (4045273565408236318)
//   - ChooseTestState (4098979167613947533)
//   - BehSuccessTestState (4487929496627066142)
TestMasterPlan2521443078354411465::TestMasterPlan2521443078354411465(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con2521443078354411465) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
TestMasterPlan2521443078354411465::~TestMasterPlan2521443078354411465()
{
    /*PROTECTED REGION ID(dcon2521443078354411465) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 3091576485060406140
 */
std::shared_ptr<UtilityFunction> UtilityFunction2521443078354411465::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(2521443078354411465) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 846865468084822174 (846865468084822174)
 *   - Comment:
 *   - Source2Dest: ChooseTestState --> BehSuccessTestState
 *
 * Precondition: 1879497210052616817 (1879497210052616817)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in ChooseTestState:
 */
bool PreCondition1879497210052616817::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(846865468084822174) ENABLED START*/
    std::cout << "The PreCondition 1879497210052616817 in Transition '846865468084822174' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 2550214909296500141 (2550214909296500141)
 *   - Comment:
 *   - Source2Dest: ChooseTestState --> SimpleTestPlanState
 *
 * Precondition: 2616157902346364992 (2616157902346364992)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in ChooseTestState:
 */
bool PreCondition2616157902346364992::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(2550214909296500141) ENABLED START*/
    std::cout << "The PreCondition 2616157902346364992 in Transition '2550214909296500141' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 2841206023261337744 (2841206023261337744)
 *   - Comment:
 *   - Source2Dest: ChooseTestState --> PlanSuccessTestState
 *
 * Precondition: 3883605426713053219 (3883605426713053219)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in ChooseTestState:
 */
bool PreCondition3883605426713053219::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(2841206023261337744) ENABLED START*/
    std::cout << "The PreCondition 3883605426713053219 in Transition '2841206023261337744' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 3219105289339324342 (3219105289339324342)
 *   - Comment:
 *   - Source2Dest: ChooseTestState --> PlanIsSuccess
 *
 * Precondition: 4584434546591332490 (4584434546591332490)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in ChooseTestState:
 */
bool PreCondition4584434546591332490::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(3219105289339324342) ENABLED START*/
    std::cout << "The PreCondition 4584434546591332490 in Transition '3219105289339324342' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 3312785896666601377 (3312785896666601377)
 *   - Comment:
 *   - Source2Dest: ChooseTestState --> RunBehaviourInSimplePlanState
 *
 * Precondition: 4585303539252259897 (4585303539252259897)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in ChooseTestState:
 */
bool PreCondition4585303539252259897::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(3312785896666601377) ENABLED START*/
    std::cout << "The PreCondition 4585303539252259897 in Transition '3312785896666601377' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 4120890224163547783 (4120890224163547783)
 *   - Comment:
 *   - Source2Dest: ChooseTestState --> MultiPlanInstanceSuccessTestState
 *
 * Precondition: 2733591692277574870 (2733591692277574870)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in ChooseTestState:
 */
bool PreCondition2733591692277574870::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(4120890224163547783) ENABLED START*/
    std::cout << "The PreCondition 2733591692277574870 in Transition '4120890224163547783' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 4209515495416542644 (4209515495416542644)
 *   - Comment:
 *   - Source2Dest: ChooseTestState --> RuntimeConditionCalledPlanState
 *
 * Precondition: 685495107222979467 (685495107222979467)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in ChooseTestState:
 */
bool PreCondition685495107222979467::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(4209515495416542644) ENABLED START*/
    std::cout << "The PreCondition 685495107222979467 in Transition '4209515495416542644' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods2521443078354411465) ENABLED START*/
// Add additional options here
void TestMasterPlan2521443078354411465::onInit() {}
/*PROTECTED REGION END*/
} // namespace alica
