#include <alica_tests/RuntimeConditionCalledPlan3213121947038933654.h>
/*PROTECTED REGION ID(eph3213121947038933654) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  RuntimeConditionCalledPlan (3213121947038933654)
//
// Tasks:
//   - RuntimeConditionPlanTask (3130906767676893645) (Entrypoint: 1437043949813218310)
//
// States:
//   - RuntimeConditionCalledPlanState2 (991974027058080866)
//   - RuntimeConditionCalledPlanState1 (2809370282893167096)
RuntimeConditionCalledPlan3213121947038933654::RuntimeConditionCalledPlan3213121947038933654(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con3213121947038933654) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
RuntimeConditionCalledPlan3213121947038933654::~RuntimeConditionCalledPlan3213121947038933654()
{
    /*PROTECTED REGION ID(dcon3213121947038933654) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

// Check of RuntimeCondition - (Name): RuntimeConditionPlanCondition, (ConditionString): , (Comment) :

/**
 * Available Vars:
 */
bool RunTimeCondition4595076014383940051::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(4595076014383940051) ENABLED START*/
    if (rp->getBasicPlan()->getBlackboard() == nullptr) {
        return true;
    }

    int64_t runTimeConditionCounter = LockedBlackboardRW(*(rp->getBlackboard())).get<int64_t>("runTimeConditionCounter");
    ++runTimeConditionCounter;
    LockedBlackboardRW(*(rp->getBlackboard())).set("runTimeConditionCounter", runTimeConditionCounter);
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Task: RuntimeConditionPlanTask  -> EntryPoint-ID: 1437043949813218310
 */
std::shared_ptr<UtilityFunction> UtilityFunction3213121947038933654::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(3213121947038933654) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 98615919776602683 (98615919776602683)
 *   - Comment:
 *   - Source2Dest: RuntimeConditionCalledPlanState1 --> RuntimeConditionCalledPlanState2
 *
 * Precondition: 778972856393262601 (778972856393262601)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in RuntimeConditionCalledPlanState1:
 *   - RuntimeConditionCalledPlanState1Behaviour (2580864383983173919)
 */
bool PreCondition778972856393262601::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(98615919776602683) ENABLED START*/
    std::cout << "The PreCondition 778972856393262601 in Transition '98615919776602683' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods3213121947038933654) ENABLED START*/
// Add additional options here

void RuntimeConditionCalledPlan3213121947038933654::onInit()
{
    LockedBlackboardRW bb(*(getBlackboard()));
    bb.set<PlanStatus>("targetChildStatus", PlanStatus::Success);
    bb.set("runTimeConditionCounter", 0);
}

int64_t RuntimeConditionCalledPlan3213121947038933654::getRunTimeConditionCounter()
{
    return LockedBlackboardRW(*getBlackboard()).get<int64_t>("runTimeConditionCounter");
}
/*PROTECTED REGION END*/
} // namespace alica
