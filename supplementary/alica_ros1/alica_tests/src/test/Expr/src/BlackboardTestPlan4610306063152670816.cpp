#include <alica_tests/BlackboardTestPlan4610306063152670816.h>
/*PROTECTED REGION ID(eph4610306063152670816) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  BlackboardTestPlan (4610306063152670816)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 2887699715416306214)
//
// States:
//   - WaitForTriggerState (924545140283507160)
//   - BlackboardMapConstantTestState (3364273219038396425)
//   - BlackboardTestSuccessState (3712638990152664591)
BlackboardTestPlan4610306063152670816::BlackboardTestPlan4610306063152670816(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con4610306063152670816) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
BlackboardTestPlan4610306063152670816::~BlackboardTestPlan4610306063152670816()
{
    /*PROTECTED REGION ID(dcon4610306063152670816) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 2887699715416306214
 */
std::shared_ptr<UtilityFunction> UtilityFunction4610306063152670816::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(4610306063152670816) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 464426070784430186 (464426070784430186)
 *   - Comment:
 *   - Source2Dest: WaitForTriggerState --> BlackboardMapConstantTestState
 *
 * Precondition: 1007584014468848906 (1007584014468848906)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in WaitForTriggerState:
 */
bool PreCondition1007584014468848906::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(464426070784430186) ENABLED START*/
    std::cout << "The PreCondition 1007584014468848906 in Transition '464426070784430186' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 3130108995589923014 (3130108995589923014)
 *   - Comment:
 *   - Source2Dest: BlackboardMapConstantTestState --> BlackboardTestSuccessState
 *
 * Precondition: 1204246573426302524 (1204246573426302524)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in BlackboardMapConstantTestState:
 *   - BlackboardMapConstantTestPlan (2390819177564329533)
 */
bool PreCondition1204246573426302524::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(3130108995589923014) ENABLED START*/
    std::cout << "The PreCondition 1204246573426302524 in Transition '3130108995589923014' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods4610306063152670816) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
