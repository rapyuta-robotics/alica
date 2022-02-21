#include "ActionServerExample2379894799421542548.h"
/*PROTECTED REGION ID(eph2379894799421542548) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  ActionServerExample (2379894799421542548)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1647616282106629095)
//
// States:
//   - WaitForGoal (4209576477302433246)
//   - ExecuteGoal (2119574391126023630)
ActionServerExample2379894799421542548::ActionServerExample2379894799421542548(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con2379894799421542548) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
ActionServerExample2379894799421542548::~ActionServerExample2379894799421542548()
{
    /*PROTECTED REGION ID(dcon2379894799421542548) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1647616282106629095
 */
std::shared_ptr<UtilityFunction> UtilityFunction2379894799421542548::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(2379894799421542548) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 430744406068167347 (430744406068167347)
 *   - Comment:
 *   - Source2Dest: WaitForGoal --> ExecuteGoal
 *
 * Precondition: 1886820548377048134 (1886820548377048134)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in WaitForGoal:
 */
bool PreCondition1886820548377048134::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(430744406068167347) ENABLED START*/
    std::cout << "The PreCondition 1886820548377048134 in Transition '430744406068167347' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 1354699620997961969 (1354699620997961969)
 *   - Comment:
 *   - Source2Dest: ExecuteGoal --> WaitForGoal
 *
 * Precondition: 587249152722263568 (587249152722263568)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in ExecuteGoal:
 *   - DummyImplementation (4126421719858579722)
 */
bool PreCondition587249152722263568::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1354699620997961969) ENABLED START*/
    std::cout << "The PreCondition 587249152722263568 in Transition '1354699620997961969' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods2379894799421542548) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
