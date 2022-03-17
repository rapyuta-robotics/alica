#include "PickDrop725594143882346503.h"
/*PROTECTED REGION ID(eph725594143882346503) ENABLED START*/
// Add additional options here
#include "alica_tests/AssignPayloadSummand.h"
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  PickDrop (725594143882346503)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 2603044554417791500)//   - DefaultTask (1225112227903) (Entrypoint: 1615605791478451403)
//
// States:
//   - Pick (1965586362363306757)
//   - Idle (3766678166599855988)
//   - TPToPickSpot (2867928428650937962)
//   - Drop (624744054901478287)
//   - TPToDropSpot (726015981546724416)
//   - Moved (2821606377944727817)
PickDrop725594143882346503::PickDrop725594143882346503(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con725594143882346503) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
PickDrop725594143882346503::~PickDrop725594143882346503()
{
    /*PROTECTED REGION ID(dcon725594143882346503) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

bool PickDrop725594143882346503::getApplicationEntrypointContext(std::unordered_map<int64_t, std::unordered_set<int64_t>>& entryPointMap)
{
    /*PROTECTED REGION ID(getApplicationEntrypointContext725594143882346503) ENABLED START*/
    // Add additional options here
    bool ret = entryPointMap.begin()->second.insert(1).second;
    ret |= std::next(entryPointMap.begin())->second.insert(2).second;
    ret |= std::next(entryPointMap.begin())->second.insert(3).second;
    ret |= std::next(entryPointMap.begin())->second.insert(4).second;
    ret |= std::next(entryPointMap.begin())->second.insert(5).second;
    ret |= std::next(entryPointMap.begin())->second.insert(6).second;
    ret |= std::next(entryPointMap.begin())->second.insert(7).second;
    ret |= std::next(entryPointMap.begin())->second.insert(8).second;
    return ret;
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 2603044554417791500
 * Task: DefaultTask  -> EntryPoint-ID: 1615605791478451403
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
 * Set parameters for child abstract plan Navigate4505472195947429717 of state TPToPickSpot2867928428650937962
 */
bool PlanAttachment1463506596775213702::setParameters(const Blackboard& parent_bb, Blackboard& child_bb)
{
    /*PROTECTED REGION ID(1463506596775213702) ENABLED START*/
    LockedBlackboardRW bb = LockedBlackboardRW(child_bb);
    bb.registerValue("nav_action", "PICK");
    return true;
    /*PROTECTED REGION END*/
}

/**
 * Set parameters for child abstract plan Navigate4505472195947429717 of state TPToDropSpot726015981546724416
 */
bool PlanAttachment2743493125610368794::setParameters(const Blackboard& parent_bb, Blackboard& child_bb)
{
    /*PROTECTED REGION ID(2743493125610368794) ENABLED START*/
    LockedBlackboardRW bb = LockedBlackboardRW(child_bb);
    bb.registerValue("nav_action", "DROP");
    return true;
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
    if (rp->amISuccessfulInAnyChild()) {
        std::cerr << "transition to TPToDropSpot" << std::endl;
    }
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
 *   - Navigate (4505472195947429717)
 */
bool PreCondition32970225314063392::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(3201336270766679779) ENABLED START*/
    if (rp->amISuccessfulInAnyChild()) {
        std::cerr << "transition to Pick" << std::endl;
    } else {
        std::cerr << "dont transition to pick" << std::endl;
    }
    return rp->amISuccessfulInAnyChild();
    /*PROTECTED REGION END*/
}

/**
 * Transition: 3356468674620212088 (3356468674620212088)
 *   - Comment:
 *   - Source2Dest: Drop --> Moved
 *
 * Precondition: 232513088105009661 (232513088105009661)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Drop:
 *   - Drop (3009473645416620380)
 */
bool PreCondition232513088105009661::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(3356468674620212088) ENABLED START*/
    if (rp->amISuccessfulInAnyChild()) {
        std::cerr << "transition to Moved" << std::endl;
    }
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
 *   - Navigate (4505472195947429717)
 */
bool PreCondition3691801807787093963::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1537785163841820841) ENABLED START*/
    if (rp->amISuccessfulInAnyChild()) {
        std::cerr << "transition to Drop" << std::endl;
    }
    return rp->amISuccessfulInAnyChild();
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods725594143882346503) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
