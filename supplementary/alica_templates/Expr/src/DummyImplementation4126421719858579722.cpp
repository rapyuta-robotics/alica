#include "DummyImplementation4126421719858579722.h"
/*PROTECTED REGION ID(eph4126421719858579722) ENABLED START*/
// Add additional options here
#include <engine/PlanInterface.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  DummyImplementation (4126421719858579722)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 2593906152181871206)
//
// States:
//   - DummyState (400856976447099508)
//   - DummyFailure (1069042249878174940)
//   - DummySuccess (1823707990536087965)
DummyImplementation4126421719858579722::DummyImplementation4126421719858579722(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con4126421719858579722) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
DummyImplementation4126421719858579722::~DummyImplementation4126421719858579722()
{
    /*PROTECTED REGION ID(dcon4126421719858579722) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 2593906152181871206
 */
std::shared_ptr<UtilityFunction> UtilityFunction4126421719858579722::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(4126421719858579722) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 161866564931915050 (161866564931915050)
 *   - Comment:
 *   - Source2Dest: DummyState --> DummyFailure
 *
 * Precondition: 2084505765197602547 (2084505765197602547)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in DummyState:
 */
bool PreCondition2084505765197602547::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(161866564931915050) ENABLED START*/
    // Add failure condition if necessary
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 293344705861516112 (293344705861516112)
 *   - Comment:
 *   - Source2Dest: DummyState --> DummySuccess
 *
 * Precondition: 3469760593538210700 (3469760593538210700)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in DummyState:
 */
bool PreCondition3469760593538210700::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(293344705861516112) ENABLED START*/
    // Add success condition
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods4126421719858579722) ENABLED START*/
// Add additional options here
void DummyImplementation4126421719858579722::run(void* msg)
{
    auto context = getPlanContext();
    if (!context.isValid()) {
        return;
    }

    LockedBlackboardRW bb = LockedBlackboardRW(*(context.getRunningPlan()->getParent()->getBasicPlan()->getBlackboard()));

    if (bb.get<std::optional<int32_t>>("goal").has_value()) {
        _currentGoal = bb.get<std::optional<int32_t>>("goal").value();
        bb.get<std::optional<int32_t>>("goal") = std::nullopt;
    }

    if (!bb.get<std::optional<int32_t>>("result").has_value()) {
        bb.get<std::optional<int32_t>>("result") = 14;
    }
}
/*PROTECTED REGION END*/
} // namespace alica
