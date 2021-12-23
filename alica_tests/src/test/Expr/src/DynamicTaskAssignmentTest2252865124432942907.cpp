#include "DynamicTaskAssignmentTest2252865124432942907.h"
/*PROTECTED REGION ID(eph2252865124432942907) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  DynamicTaskAssignmentTest (2252865124432942907)
//
// Tasks:
//   - DynamicTask (1163169622598227531) (Entrypoint: 3150793708487666867)
//
// States:
//   - DynamicTaskFinished (2788356913272296281)
//   - DynamicState1 (2800951832651805821)
DynamicTaskAssignmentTest2252865124432942907::DynamicTaskAssignmentTest2252865124432942907(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con2252865124432942907) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
DynamicTaskAssignmentTest2252865124432942907::~DynamicTaskAssignmentTest2252865124432942907()
{
    /*PROTECTED REGION ID(dcon2252865124432942907) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

bool DynamicTaskAssignmentTest2252865124432942907::getApplicationEntrypointContext(std::unordered_map<int64_t, std::unordered_set<int64_t>>& entryPointMap)
{
    /*PROTECTED REGION ID(getApplicationEntrypointContext2252865124432942907) ENABLED START*/
    // Add additional options here
    return entryPointMap.begin()->second.insert(1).second;
    /*PROTECTED REGION END*/
}

/**
 * Task: DynamicTask  -> EntryPoint-ID: 3150793708487666867
 */
std::shared_ptr<UtilityFunction> UtilityFunction2252865124432942907::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(2252865124432942907) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 1813059625501892456 (1813059625501892456)
 *   - Comment:
 *   - Source2Dest: DynamicState1 --> DynamicTaskFinished
 *
 * Precondition: 1078898265232036813 (1078898265232036813)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in DynamicState1:
 *   - DynamicTaskBehavior (4044546549214673470)
 */
bool PreCondition1078898265232036813::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1813059625501892456) ENABLED START*/
    std::cout << "The PreCondition 1078898265232036813 in Transition '1813059625501892456' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods2252865124432942907) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
