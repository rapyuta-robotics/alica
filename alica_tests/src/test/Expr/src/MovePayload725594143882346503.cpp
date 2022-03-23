#include "MovePayload725594143882346503.h"
/*PROTECTED REGION ID(eph725594143882346503) ENABLED START*/
// Add additional options here
#include "alica_tests/AssignPayloadSummand.h"
#include "alica_tests/TaskInstantiationIntegrationWorldModel.h"
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  MovePayload (725594143882346503)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1615605791478451403)//   - DefaultTask (1225112227903) (Entrypoint: 2603044554417791500)
//
// States:
//   - Drop (624744054901478287)
//   - TPToDropSpot (726015981546724416)
//   - Pick (1965586362363306757)
//   - TPToPickSpot (2867928428650937962)
//   - Moved (3464891834530950837)
//   - Idle (3766678166599855988)
//   - AssignPayload (3785266111914580157)
MovePayload725594143882346503::MovePayload725594143882346503(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con725594143882346503) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
MovePayload725594143882346503::~MovePayload725594143882346503()
{
    /*PROTECTED REGION ID(dcon725594143882346503) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

bool MovePayload725594143882346503::getApplicationEntrypointContext(std::unordered_map<int64_t, std::unordered_set<int64_t>>& entryPointMap)
{
    /*PROTECTED REGION ID(getApplicationEntrypointContext725594143882346503) ENABLED START*/
    // Add additional options here
    bool ret = entryPointMap[2603044554417791500].insert(1).second;
    ret |= entryPointMap[2603044554417791500].insert(2).second;
    ret |= entryPointMap[2603044554417791500].insert(3).second;
    ret |= entryPointMap[2603044554417791500].insert(4).second;
    ret |= entryPointMap[2603044554417791500].insert(5).second;
    ret |= entryPointMap[2603044554417791500].insert(6).second;
    ret |= entryPointMap[2603044554417791500].insert(7).second;
    ret |= entryPointMap[2603044554417791500].insert(8).second;
    return ret;
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1615605791478451403
 * Task: DefaultTask  -> EntryPoint-ID: 2603044554417791500
 */
std::shared_ptr<UtilityFunction> UtilityFunction725594143882346503::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(725594143882346503) ENABLED START*/
    std::shared_ptr<UtilityFunction> function = std::make_shared<DefaultUtilityFunction>(plan);

    AssignPayloadSummand* us = new AssignPayloadSummand(1.0);
    us->addEntryPoint(plan->getEntryPointByID(2603044554417791500));
    us->addEntryPoint(plan->getEntryPointByID(1615605791478451403));

    function->editUtilSummands().emplace_back(us);
    return function;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 2630758425967053453 (2630758425967053453)
 *   - Comment:
 *   - Source2Dest: Drop --> Moved
 *
 * Precondition: 2187308102082241829 (2187308102082241829)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Drop:
 *   - Drop (3009473645416620380)
 */
bool PreCondition2187308102082241829::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(2630758425967053453) ENABLED START*/
    return rp->amISuccessfulInAnyChild();
    /*PROTECTED REGION END*/
}

/**
 * Transition: 1537785163841820841 (1537785163841820841)
 *   - Comment:
 *   - Source2Dest: TPToDropSpot --> Drop
 *
 * Precondition: 3691801807787093963 (3691801807787093963)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in TPToDropSpot:
 *   - NavigateToDrop (4459885370764933844)
 */
bool PreCondition3691801807787093963::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1537785163841820841) ENABLED START*/
    return rp->amISuccessfulInAnyChild();
    /*PROTECTED REGION END*/
}

/**
 * Transition: 1425389364493331507 (1425389364493331507)
 *   - Comment:
 *   - Source2Dest: Pick --> TPToDropSpot
 *
 * Precondition: 3953991713597643491 (3953991713597643491)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Pick:
 *   - Pick (2580816776008671737)
 */
bool PreCondition3953991713597643491::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1425389364493331507) ENABLED START*/
    return rp->amISuccessfulInAnyChild();
    /*PROTECTED REGION END*/
}

/**
 * Transition: 3201336270766679779 (3201336270766679779)
 *   - Comment:
 *   - Source2Dest: TPToPickSpot --> Pick
 *
 * Precondition: 32970225314063392 (32970225314063392)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in TPToPickSpot:
 *   - NavigateToPick (4505472195947429717)
 */
bool PreCondition32970225314063392::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(3201336270766679779) ENABLED START*/
    return rp->amISuccessfulInAnyChild();
    /*PROTECTED REGION END*/
}

/**
 * Transition: 3103663386312740882 (3103663386312740882)
 *   - Comment:
 *   - Source2Dest: AssignPayload --> TPToPickSpot
 *
 * Precondition: 1971173312201839855 (1971173312201839855)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in AssignPayload:
 *   - AssignPayload (3826644292150922713)
 */
bool PreCondition1971173312201839855::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(3103663386312740882) ENABLED START*/
    return rp->amISuccessfulInAnyChild();
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods725594143882346503) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
