#include "ConstraintCreator.h"

#include "Authority/constraints/AuthorityTest1414403413451Constraints.h"
#include "Authority/constraints/AuthorityTestMaster1414403396328Constraints.h"
#include "Behaviour/constraints/AlwaysFail1532424188199Constraints.h"
#include "Behaviour/constraints/Attack1402488848841Constraints.h"
#include "Behaviour/constraints/AttackOpp1402489351885Constraints.h"
#include "Behaviour/constraints/BehAAA1629895901559Constraints.h"
#include "Behaviour/constraints/BehBAA1629895911592Constraints.h"
#include "Behaviour/constraints/ConstraintUsingBehaviour1414068597716Constraints.h"
#include "Behaviour/constraints/CountIndefinitely1529456643148Constraints.h"
#include "Behaviour/constraints/DefendMid1402488730695Constraints.h"
#include "Behaviour/constraints/EmptyBehaviour1625610857563Constraints.h"
#include "Behaviour/constraints/MidFieldStandard1402488696205Constraints.h"
#include "Behaviour/constraints/NotToTrigger1429017274116Constraints.h"
#include "Behaviour/constraints/ReadConfigurationBehaviour1588061129360Constraints.h"
#include "Behaviour/constraints/SuccessSpam1522377401286Constraints.h"
#include "Behaviour/constraints/Tackle1402488939130Constraints.h"
#include "Behaviour/constraints/TriggerA1428508297492Constraints.h"
#include "Behaviour/constraints/TriggerB1428508316905Constraints.h"
#include "Behaviour/constraints/TriggerC1428508355209Constraints.h"
#include "Configurations/constraints/ConfigurationTestPlan1588060981661Constraints.h"
#include "Configurations/constraints/ReadConfInPlantype1588061801734Constraints.h"
#include "Configurations/constraints/ReadConfigurationPlan1588061334567Constraints.h"
#include "constraints/AdjacentSuccessMasterPlan3254486013443203397Constraints.h"
#include "constraints/AdjacentSuccessSubPlan1682631238618360548Constraints.h"
#include "constraints/AssignPayload3826644292150922713Constraints.h"
#include "constraints/AttackPlan1402488634525Constraints.h"
#include "constraints/BackForth1529456584982Constraints.h"
#include "constraints/BehaviorSuccessSpamMaster1522377375148Constraints.h"
#include "constraints/BehaviourTriggerTestPlan1428508768572Constraints.h"
#include "constraints/ConstraintTestMaster1414068495566Constraints.h"
#include "constraints/ConstraintTestPlan1414068524245Constraints.h"
#include "constraints/Defend1402488893641Constraints.h"
#include "constraints/Drop3009473645416620380Constraints.h"
#include "constraints/DynamicTaskAssignmentTest2252865124432942907Constraints.h"
#include "constraints/DynamicTaskAssignmentTestMaster1602078208698393838Constraints.h"
#include "constraints/DynamicTaskBehavior4044546549214673470Constraints.h"
#include "constraints/DynamicTaskBehaviourLD19516698765703926Constraints.h"
#include "constraints/DynamicTaskLA3337489358878214836Constraints.h"
#include "constraints/DynamicTaskLB4316676367342780557Constraints.h"
#include "constraints/DynamicTaskLC2140075868731779222Constraints.h"
#include "constraints/DynamicTaskTogether1338298120374694644Constraints.h"
#include "constraints/EmptyPlan984284423749038756Constraints.h"
#include "constraints/ExecuteBehaviourInSubPlan3172561495666303184Constraints.h"
#include "constraints/FailsOnOne1530069246103Constraints.h"
#include "constraints/FreePayload422054015709952219Constraints.h"
#include "constraints/FrequencyTestPlan1626848999740Constraints.h"
#include "constraints/GoalPlan1402488870347Constraints.h"
#include "constraints/HandleFailExplicit1530004915640Constraints.h"
#include "constraints/HandleFailExplicitMaster1530004940652Constraints.h"
#include "constraints/MasterPlan1402488437260Constraints.h"
#include "constraints/MasterPlanTaskAssignment1407152758497Constraints.h"
#include "constraints/MasterPlanTestConditionPlanType1418042656594Constraints.h"
#include "constraints/MasterSyncTransition1418825395939Constraints.h"
#include "constraints/MidFieldPlayPlan1402488770050Constraints.h"
#include "constraints/MultiAgentTestMaster1413200842973Constraints.h"
#include "constraints/MultiAgentTestPlan1413200862180Constraints.h"
#include "constraints/NavigateToDrop4459885370764933844Constraints.h"
#include "constraints/NavigateToPick4505472195947429717Constraints.h"
#include "constraints/OrderedSchedulingTestPlan1629895582410Constraints.h"
#include "constraints/OtherPlan1418042819203Constraints.h"
#include "constraints/Pick2580816776008671737Constraints.h"
#include "constraints/PickDrop725594143882346503Constraints.h"
#include "constraints/PlanA1629895837159Constraints.h"
#include "constraints/PlanAA1629895864090Constraints.h"
#include "constraints/PlanB1629895853508Constraints.h"
#include "constraints/PlanBA1629895873188Constraints.h"
#include "constraints/PlanFive1407153703092Constraints.h"
#include "constraints/PlanFour1407153683051Constraints.h"
#include "constraints/PlanOne1407153611768Constraints.h"
#include "constraints/PlanPoolTestMasterPlan1964838032551226161Constraints.h"
#include "constraints/PlanPoolTestSubPlan432995127772554364Constraints.h"
#include "constraints/PlanThree1407153663917Constraints.h"
#include "constraints/PlanTwo1407153645238Constraints.h"
#include "constraints/PreConditionPlan1418042796751Constraints.h"
#include "constraints/RealMasterPlanForSyncTest1418902217839Constraints.h"
#include "constraints/RuntimeConditionPlan1418042806575Constraints.h"
#include "constraints/SchedulingTestMasterPlan1613378382024Constraints.h"
#include "constraints/SchedulingTestPlan11613378406860Constraints.h"
#include "constraints/SchedulingTestPlan21613378423610Constraints.h"
#include "constraints/SchedulingTestPlan31613378433623Constraints.h"
#include "constraints/SchedulingTestSequencePlan11614963946725Constraints.h"
#include "constraints/SchedulingTestSequenceSubPlan11614964379654Constraints.h"
#include "constraints/SchedulingTestSequenceSubPlan21614964444419Constraints.h"
#include "constraints/SchedulingTestSequenceSubPlan31614964478264Constraints.h"
#include "constraints/SerializationMasterPlan373109241446504968Constraints.h"
#include "constraints/SerializationSubPlanA1433931143598606082Constraints.h"
#include "constraints/SerializationSubPlanB230205985761632608Constraints.h"
#include "constraints/SerializationSubPlanC2359124678252958039Constraints.h"
#include "constraints/SerializationSubPlanD1781630225028158279Constraints.h"
#include "constraints/SimpleTestPlan1412252439925Constraints.h"
#include "constraints/Tackle1402489318663Constraints.h"
#include "constraints/TaskInstantiationIntegrationTestMaster4603312216886200747Constraints.h"
#include "constraints/TestBehaviour55178365253414982Constraints.h"
#include "constraints/TestParameterPassing1692837668719979457Constraints.h"
#include "constraints/TestParameterPassingBehaviour831400441334251602Constraints.h"
#include "constraints/TestParameterPassingMaster1179066429431332055Constraints.h"
#include "constraints/TestTracingMasterPlan691392966514374878Constraints.h"
#include "constraints/TestTracingSubPlan1482512794732634139Constraints.h"

#include <iostream>

namespace alica
{

ConstraintCreator::ConstraintCreator() {}

ConstraintCreator::~ConstraintCreator() {}

std::shared_ptr<BasicConstraint> ConstraintCreator::createConstraint(int64_t constraintConfId)
{
    switch (constraintConfId) {
    case 1402489460549:
        return std::make_shared<Constraint1402489460549>();
        break;
    case 1402489462088:
        return std::make_shared<Constraint1402489462088>();
        break;
    case 1403773741874:
        return std::make_shared<Constraint1403773741874>();
        break;
    case 1414068566297:
        return std::make_shared<Constraint1414068566297>();
        break;
    default:
        std::cerr << "ConstraintCreator: Unknown constraint requested: " << constraintConfId << std::endl;
        throw new std::exception();
        break;
    }
}

} // namespace alica
