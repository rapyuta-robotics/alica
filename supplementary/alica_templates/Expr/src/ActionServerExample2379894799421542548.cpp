#include "ActionServerExample2379894799421542548.h"
/*PROTECTED REGION ID(eph2379894799421542548) ENABLED START*/
// Add additional options here
#include <string>
#include <optional>
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
    _actionServer = std::make_unique<actionlib::SimpleActionServer<alica_templates::DummyAction>>(_nh, std::string("DummyActionServer"), false);
    _actionServer->registerGoalCallback(std::bind(&ActionServerExample2379894799421542548::goalCallback, this));
    _actionServer->registerPreemptCallback(std::bind(&ActionServerExample2379894799421542548::preemptCallback, this));
    _actionServer->start();
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
    bb.get<std::optional<int32_t>>("goal") = _actionServer->acceptNewGoal()->value;
}

void ActionServerExample2379894799421542548::preemptCallback()
{
    LockedBlackboardRW bb = LockedBlackboardRW(*getBlackboard());
    bb.get<std::optional<int32_t>>("cancel") = true;
}

void ActionServerExample2379894799421542548::onInit()
{
    LockedBlackboardRW bb = LockedBlackboardRW(*getBlackboard());
    bb.registerValue("result", std::optional<int32_t>());
    bb.registerValue("feedback", std::optional<int32_t>());
    bb.registerValue("goal", std::optional<int32_t>());
    bb.registerValue("cancel", std::optional<bool>());
    bb.registerValue("cancelAccepted", std::optional<bool>());
}

void ActionServerExample2379894799421542548::run(void* msg)
{
    LockedBlackboardRW bb = LockedBlackboardRW(*getBlackboard());
    if (bb.get<std::optional<int32_t>>("result") != std::nullopt) {
        int32_t resultValue = *(bb.get<std::optional<int32_t>>("result"));
        _result.value = resultValue;
        _actionServer->setSucceeded(_result);
        bb.get<std::optional<int32_t>>("result") = std::nullopt;
    }
    if (bb.get<std::optional<std::vector<int32_t>>>("feedback") != std::nullopt) {
        int32_t feedbackValue = *(bb.get<std::optional<int32_t>>("feedback"));
        _feedback.value = feedbackValue;
        _actionServer->publishFeedback(_feedback);
        bb.get<std::optional<alica_templates::DummyActionFeedback>>("feedback") = std::nullopt;
    }
    if (bb.get<std::optional<bool>>("cancelAccepted") != std::nullopt) {
        int32_t resultValue = *(bb.get<std::optional<int32_t>>("result"));
        _result.value = resultValue;
        _actionServer->setAborted(_result);
        bb.get<std::optional<bool>>("cancelAccepted") = std::nullopt;
        bb.get<std::optional<int32_t>>("result") = std::nullopt;
    }
}
/*PROTECTED REGION END*/
} // namespace alica
