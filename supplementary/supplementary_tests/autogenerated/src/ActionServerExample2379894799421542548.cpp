#include "ActionServerExample2379894799421542548.h"
/*PROTECTED REGION ID(eph2379894799421542548) ENABLED START*/
// Add additional options here
#include <string>
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
    auto plan = rp->getBasicPlan();
    LockedBlackboardRO bb = LockedBlackboardRO(*(rp->getBasicPlan()->getBlackboard()));
    return bb.get<std::optional<int32_t>>("goal") != std::nullopt;
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
    auto plan = rp->getBasicPlan();
    return rp->isAnyChildTaskSuccessful() || rp->isAnyChildStatus(PlanStatus::Failed);
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods2379894799421542548) ENABLED START*/
// Add additional options here
void ActionServerExample2379894799421542548::goalCallback()
{
    LockedBlackboardRW bb = LockedBlackboardRW(*getBlackboard());
    bb.get<std::optional<int32_t>>("_goalFinished") = std::nullopt;
    bb.get<std::optional<int32_t>>("goal") = _actionServer.acceptNewGoal();
}

void ActionServerExample2379894799421542548::preemptCallback()
{
    LockedBlackboardRW bb = LockedBlackboardRW(*getBlackboard());
    bb.get<std::optional<int32_t>>("cancel") = true;
}

void ActionServerExample2379894799421542548::onInit()
{
    LockedBlackboardRW bb = LockedBlackboardRW(*getBlackboard());
    bb.registerValue("result", std::optional<std::vector<int32_t>>());
    bb.registerValue("feedback", std::optional<std::vector<int32_t>>());
    bb.registerValue("goal", std::optional<int32_t>());
    bb.registerValue("cancel", std::optional<bool>());
    bb.registerValue("cancelAccepted", std::optional<bool>());
    _actionServer(_nh, std::string("DummyActionServer"), false);
    _actionServer.registerGoalCallback(std::bind(&ActionServerExample2379894799421542548::goalCallback), this);
    _actionServer.registerPreemptCallback(std::bind(&ActionServerExample2379894799421542548::preemptCallback), this);
    _actionServer.start();
}

void ActionServerExample2379894799421542548::run(void* msg)
{
    LockedBlackboardRW bb = LockedBlackboardRW(*getBlackboard());
    if (bb.get<std::optional<std::vector<int32_t>>>("result") != std::nullopt) {
        _actionServer.setSucceeded(bb.get<std::optional<supplementary_tests::DummyActionResult>>("result"));
        bb.get<std::optional<std::vector<int32_t>>>("result") = std::nullopt;
    }
    if (bb.get<std::optional<std::vector<int32_t>>>("feedback") != std::nullopt) {
        _actionServer.publishFeedback(bb.get<std::optional<std::vector<int32_t>>>("feedback"));
        bb.get<std::optional<std::vector<int32_t>>>("feedback") = std::nullopt;
    }
    if (bb.get<std::optional<bool>>("cancelAccepted") != std::nullopt) {
        _actionServer.setAborted(bb.get<std::optional<supplementary_tests::DummyActionResult>>("result"));
        bb.get<std::optional<bool>>("cancelAccepted") = std::nullopt;
        bb.get<std::optional<supplementary_tests::DummyActionResult>("result") = std::nullopt;
    }
}

void ActionServerExample2379894799421542548::onTerminate()
{
    _actionServer.shutdown();
}
/*PROTECTED REGION END*/
} // namespace alica
