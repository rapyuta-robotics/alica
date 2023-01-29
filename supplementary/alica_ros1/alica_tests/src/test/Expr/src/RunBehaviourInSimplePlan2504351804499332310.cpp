#include <alica_tests/RunBehaviourInSimplePlan2504351804499332310.h>
/*PROTECTED REGION ID(eph2504351804499332310) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  RunBehaviourInSimplePlan (2504351804499332310)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 2768515147950934595)
//
// States:
//   - State2 (681938134974956178)
//   - State1 (1006522403402265538)
RunBehaviourInSimplePlan2504351804499332310::RunBehaviourInSimplePlan2504351804499332310(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con2504351804499332310) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
RunBehaviourInSimplePlan2504351804499332310::~RunBehaviourInSimplePlan2504351804499332310()
{
    /*PROTECTED REGION ID(dcon2504351804499332310) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): , (Comment) :

/**
 * Available Vars:
 */
bool RunTimeCondition4404788800584486714::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(4404788800584486714) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 2768515147950934595
 */
std::shared_ptr<UtilityFunction> UtilityFunction2504351804499332310::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(2504351804499332310) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 1055311237496664394 (1055311237496664394)
 *   - Comment:
 *   - Source2Dest: State1 --> State2
 *
 * Precondition: 122747038863060590 (122747038863060590)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in State1:
 *   - State1Behaviour (3563417394101512880)
 */
bool PreCondition122747038863060590::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(1055311237496664394) ENABLED START*/
    return rp->isAnyChildStatus(PlanStatus::Success);
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods2504351804499332310) ENABLED START*/
// Add additional options here
void RunBehaviourInSimplePlan2504351804499332310::onInit()
{
    LockedBlackboardRW bb(*(getBlackboard()));
    bb.set<PlanStatus>("targetChildStatus", PlanStatus::Success);
    bb.set("runTimeConditionCounter", 0);
}

int64_t RunBehaviourInSimplePlan2504351804499332310::getRunTimeConditionCounter()
{
    return LockedBlackboardRW(*getBlackboard()).get<int64_t>("runTimeConditionCounter");
}
/*PROTECTED REGION END*/
} // namespace alica
