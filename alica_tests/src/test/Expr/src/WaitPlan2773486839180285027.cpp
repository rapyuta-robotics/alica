#include "WaitPlan2773486839180285027.h"
/*PROTECTED REGION ID(eph2773486839180285027) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  WaitPlan (2773486839180285027)
//
// Tasks:
//   - Attack (1222613952469) (Entrypoint: 13426738844110157)
//
// States:
//   - Suc (699161635959867032)
//   - Wait (1909100645626369899)
WaitPlan2773486839180285027::WaitPlan2773486839180285027()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con2773486839180285027) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
WaitPlan2773486839180285027::~WaitPlan2773486839180285027()
{
    /*PROTECTED REGION ID(dcon2773486839180285027) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: Attack  -> EntryPoint-ID: 13426738844110157
 */
std::shared_ptr<UtilityFunction> UtilityFunction2773486839180285027::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(2773486839180285027) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
/**
 * Transition: 1509957282302076977 (1509957282302076977)
 *   - Comment:
 *   - Source2Dest: Wait --> Suc
 *
 * Precondition: ToSuccess (3266818544279107129)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Wait:
 */
bool PreCondition3266818544279107129::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1509957282302076977) ENABLED START*/
    std::cout << "The PreCondition 3266818544279107129 in Transition '1509957282302076977' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods2773486839180285027) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
