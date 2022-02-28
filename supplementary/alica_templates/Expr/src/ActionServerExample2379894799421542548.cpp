#include "ActionServerExample2379894799421542548.h"
/*PROTECTED REGION ID(eph2379894799421542548) ENABLED START*/
// Add additional options here
#include <optional>
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
    LockedBlackboardRO bb = LockedBlackboardRO(*(rp->getBasicPlan()->getBlackboard()));
    return bb.get<std::optional<int32_t>>("goal").has_value();
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
    LockedBlackboardRO bb = LockedBlackboardRO(*(rp->getBasicPlan()->getBlackboard()));
    return rp->amISuccessfulInAnyChild() || rp->isAnyChildStatus(PlanStatus::Failed) || bb.get<std::optional<bool>>("cancel").value_or(false);
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods2379894799421542548) ENABLED START*/
// Add additional options here
void ActionServerExample2379894799421542548::onInit()
{
    LockedBlackboardRW bb = LockedBlackboardRW(*getBlackboard());
    bb.registerValue("result", std::optional<int32_t>());
    bb.registerValue("feedback", std::optional<int32_t>());
    bb.registerValue("goal", std::optional<int32_t>());
    bb.registerValue("cancel", std::optional<bool>());
    // To increase the lifestime of the server, create the server in the plan's constructor and don't destroy it in onTerminate()
    _actionServer = std::make_unique<actionlib::SimpleActionServer<alica_msgs::DummyAction>>(_nh, std::string("DummyActionServer"), true);
}

void ActionServerExample2379894799421542548::run(void* msg)
{
    LockedBlackboardRW bb = LockedBlackboardRW(*getBlackboard());
    if (_actionServer->isNewGoalAvailable()) {
        bb.get<std::optional<int32_t>>("cancel") = std::nullopt;
        // on the arrival of a new goal, preempt the current goal
        if (_actionServer->isActive()) {
            _actionServer->setPreempted();
        }
        // dont send result of the old goal when a new goal is available
        bb.get<std::optional<int32_t>>("result") = std::nullopt;
        bb.get<std::optional<int32_t>>("goal") = _actionServer->acceptNewGoal()->value;
    }
    if (_actionServer->isPreemptRequested()) {
        _actionServer->setAborted();
        // dont send result of the cancelled goal, clear cancelled goal
        bb.get<std::optional<int32_t>>("result") = std::nullopt;
        bb.get<std::optional<int32_t>>("goal") = std::nullopt;
        bb.get<std::optional<int32_t>>("cancel") = true;
    }
    if (bb.get<std::optional<int32_t>>("result").has_value()) {
        int32_t resultValue = *(bb.get<std::optional<int32_t>>("result"));
        _result.value = resultValue;
        _actionServer->setSucceeded(_result);
        bb.get<std::optional<int32_t>>("result") = std::nullopt;
        // dont send feedback after sending the result
        bb.get<std::optional<int32_t>>("feedback") = std::nullopt;
    }
    if (bb.get<std::optional<int32_t>>("feedback").has_value()) {
        int32_t feedbackValue = *(bb.get<std::optional<int32_t>>("feedback"));
        _feedback.value = feedbackValue;
        _actionServer->publishFeedback(_feedback);
        bb.get<std::optional<int32_t>>("feedback") = std::nullopt;
    }
}

void ActionServerExample2379894799421542548::onTerminate()
{
    _actionServer.reset(); // will shutdown and destroy the action server
}
/*PROTECTED REGION END*/
} // namespace alica
