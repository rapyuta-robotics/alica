#include <alica_tests/BlackboardMapConstantTestPlan2390819177564329533.h>
/*PROTECTED REGION ID(eph2390819177564329533) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  BlackboardMapConstantTestPlan (2390819177564329533)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1019181667380114069)
//
// States:
//   - InitState (602978301530919579)
//   - BlackboardMapConstantTestSuccess (4000951790164702175)
//   - MapConstantTestState (2937672201731359801)
BlackboardMapConstantTestPlan2390819177564329533::BlackboardMapConstantTestPlan2390819177564329533(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con2390819177564329533) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
BlackboardMapConstantTestPlan2390819177564329533::~BlackboardMapConstantTestPlan2390819177564329533()
{
    /*PROTECTED REGION ID(dcon2390819177564329533) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1019181667380114069
 */
std::shared_ptr<UtilityFunction> UtilityFunction2390819177564329533::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(2390819177564329533) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 1814492430994326017 (1814492430994326017)
 *   - Comment:
 *   - Source2Dest: InitState --> MapConstantTestState
 *
 * Precondition: 170416006263862082 (170416006263862082)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in InitState:
 *   - IncreaseCountByX (1084111613399827667)
 */
bool PreCondition170416006263862082::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(1814492430994326017) ENABLED START*/
    std::cout << "The PreCondition 170416006263862082 in Transition '1814492430994326017' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 1658993097517406800 (1658993097517406800)
 *   - Comment:
 *   - Source2Dest: MapConstantTestState --> BlackboardMapConstantTestSuccess
 *
 * Precondition: 3743250903768144590 (3743250903768144590)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in MapConstantTestState:
 *   - IncreaseCountByX (1084111613399827667)
 */
bool PreCondition3743250903768144590::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(1658993097517406800) ENABLED START*/
    std::cout << "The PreCondition 3743250903768144590 in Transition '1658993097517406800' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods2390819177564329533) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
