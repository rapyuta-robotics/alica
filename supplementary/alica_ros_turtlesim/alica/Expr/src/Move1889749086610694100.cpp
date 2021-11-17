#include "Move1889749086610694100.h"
/*PROTECTED REGION ID(eph1889749086610694100) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  Move (1889749086610694100)
//
// Tasks:
//   - Follower (3759439551323513525) (Entrypoint: 3277312192440194145)//   - Leader (826983480584534597) (Entrypoint: 4346694000146342467)
//
// States:
//   - AlignCircle (2299237921449867536)
//   - Move2Center (4158797811607100614)
Move1889749086610694100::Move1889749086610694100()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1889749086610694100) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
Move1889749086610694100::~Move1889749086610694100()
{
    /*PROTECTED REGION ID(dcon1889749086610694100) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

// Check of RuntimeCondition - (Name): CircleRuntimeCondition, (ConditionString): , (Comment) :

/**
 * Available Vars:
 */
bool RunTimeCondition1288817888979746811::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1288817888979746811) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Task: Follower  -> EntryPoint-ID: 3277312192440194145
 * Task: Leader  -> EntryPoint-ID: 4346694000146342467
 */
std::shared_ptr<UtilityFunction> UtilityFunction1889749086610694100::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1889749086610694100) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1889749086610694100) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
