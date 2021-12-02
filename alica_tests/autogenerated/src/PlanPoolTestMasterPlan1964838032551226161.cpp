#include "PlanPoolTestMasterPlan1964838032551226161.h"
/*PROTECTED REGION ID(eph1964838032551226161) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  PlanPoolTestMasterPlan (1964838032551226161)
//
// Tasks:
//   - Attack (1222613952469) (Entrypoint: 4019498150183138248)
//
// States:
//   - 508968687272454527 (508968687272454527)
//   - 807253925929611286 (807253925929611286)
PlanPoolTestMasterPlan1964838032551226161::PlanPoolTestMasterPlan1964838032551226161()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1964838032551226161) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
PlanPoolTestMasterPlan1964838032551226161::~PlanPoolTestMasterPlan1964838032551226161()
{
    /*PROTECTED REGION ID(dcon1964838032551226161) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: Attack  -> EntryPoint-ID: 4019498150183138248
 */
std::shared_ptr<UtilityFunction> UtilityFunction1964838032551226161::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1964838032551226161) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
/**
 * Transition: 4186311028071767502 (4186311028071767502)
 *   - Comment:
 *   - Source2Dest: 508968687272454527 --> 807253925929611286
 *
 * Precondition: 4238964946542987247 (4238964946542987247)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in 508968687272454527:
 *   - PlanPoolTestSubPlan (432995127772554364)
 */
bool PreCondition4238964946542987247::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(4186311028071767502) ENABLED START*/
    std::cout << "The PreCondition 4238964946542987247 in Transition '4186311028071767502' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Transition: 3610919168422994279 (3610919168422994279)
 *   - Comment:
 *   - Source2Dest: 807253925929611286 --> 508968687272454527
 *
 * Precondition: 4115970455290610262 (4115970455290610262)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in 807253925929611286:
 *   - PlanPoolTestSubPlan (432995127772554364)
 */
bool PreCondition4115970455290610262::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(3610919168422994279) ENABLED START*/
    std::cout << "The PreCondition 4115970455290610262 in Transition '3610919168422994279' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1964838032551226161) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
