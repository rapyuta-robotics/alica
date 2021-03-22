#include "ConstraintCreator.h"

#include "Authority/constraints/AuthorityTest1414403413451Constraints.h"
#include "Authority/constraints/AuthorityTestMaster1414403396328Constraints.h"
#include "Behaviour/constraints/AlwaysFail1532424188199Constraints.h"
#include "Behaviour/constraints/Attack1402488848841Constraints.h"
#include "Behaviour/constraints/AttackOpp1402489351885Constraints.h"
#include "Behaviour/constraints/ConstraintUsingBehaviour1414068597716Constraints.h"
#include "Behaviour/constraints/CountIndefinitely1529456643148Constraints.h"
#include "Behaviour/constraints/DefendMid1402488730695Constraints.h"
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
#include "constraints/AttackPlan1402488634525Constraints.h"
#include "constraints/BackForth1529456584982Constraints.h"
#include "constraints/BehaviorSuccessSpamMaster1522377375148Constraints.h"
#include "constraints/BehaviourTriggerTestPlan1428508768572Constraints.h"
#include "constraints/ConstraintTestMaster1414068495566Constraints.h"
#include "constraints/ConstraintTestPlan1414068524245Constraints.h"
#include "constraints/Defend1402488893641Constraints.h"
#include "constraints/FailsOnOne1530069246103Constraints.h"
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
#include "constraints/OtherPlan1418042819203Constraints.h"
#include "constraints/PlanFive1407153703092Constraints.h"
#include "constraints/PlanFour1407153683051Constraints.h"
#include "constraints/PlanOne1407153611768Constraints.h"
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
#include "constraints/SimpleTestPlan1412252439925Constraints.h"
#include "constraints/Tackle1402489318663Constraints.h"

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
