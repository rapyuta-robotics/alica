#include "PlanCreator.h"
#include "AttackPlan.h"
#include "Authority/AuthorityTest.h"
#include "Authority/AuthorityTestMaster.h"
#include "BackForth.h"
#include "BehaviorSuccessSpamMaster.h"
#include "BehaviourTriggerTestPlan.h"
#include "Configurations/ConfigurationTestPlan.h"
#include "Configurations/ReadConfInPlantype.h"
#include "Configurations/ReadConfigurationPlan.h"
#include "ConstraintTestMaster.h"
#include "ConstraintTestPlan.h"
#include "Defend.h"
#include "FailsOnOne.h"
#include "GoalPlan.h"
#include "HandleFailExplicit.h"
#include "HandleFailExplicitMaster.h"
#include "MasterPlan.h"
#include "MasterPlanTaskAssignment.h"
#include "MasterPlanTestConditionPlanType.h"
#include "MasterSyncTransition.h"
#include "MidFieldPlayPlan.h"
#include "MultiAgentTestMaster.h"
#include "MultiAgentTestPlan.h"
#include "OtherPlan.h"
#include "PlanFive.h"
#include "PlanFour.h"
#include "PlanOne.h"
#include "PlanThree.h"
#include "PlanTwo.h"
#include "PreConditionPlan.h"
#include "RealMasterPlanForSyncTest.h"
#include "RuntimeConditionPlan.h"
#include "SimpleTestPlan.h"
#include "Tackle.h"
#include "engine/BasicPlan.h"

namespace alica
{

PlanCreator::PlanCreator() {}

PlanCreator::~PlanCreator() {}

std::shared_ptr<BasicBPlan> PlanCreator::createPlan(int64_t planId)
{
    switch (planId) {
    case 1402488437260:
        return std::make_shared<MasterPlan>();
        break;
    case 1402488634525:
        return std::make_shared<AttackPlan>();
        break;
    case 1402488770050:
        return std::make_shared<MidFieldPlayPlan>();
        break;
    case 1402488870347:
        return std::make_shared<GoalPlan>();
        break;
    case 1402488893641:
        return std::make_shared<Defend>();
        break;
    case 1402489318663:
        return std::make_shared<Tackle>();
        break;
    case 1407152758497:
        return std::make_shared<MasterPlanTaskAssignment>();
        break;
    case 1407153611768:
        return std::make_shared<PlanOne>();
        break;
    case 1407153645238:
        return std::make_shared<PlanTwo>();
        break;
    case 1407153663917:
        return std::make_shared<PlanThree>();
        break;
    case 1407153683051:
        return std::make_shared<PlanFour>();
        break;
    case 1407153703092:
        return std::make_shared<PlanFive>();
        break;
    case 1412252439925:
        return std::make_shared<SimpleTestPlan>();
        break;
    case 1413200842973:
        return std::make_shared<MultiAgentTestMaster>();
        break;
    case 1413200862180:
        return std::make_shared<MultiAgentTestPlan>();
        break;
    case 1414068495566:
        return std::make_shared<ConstraintTestMaster>();
        break;
    case 1414068524245:
        return std::make_shared<ConstraintTestPlan>();
        break;
    case 1414403396328:
        return std::make_shared<AuthorityTestMaster>();
        break;
    case 1414403413451:
        return std::make_shared<AuthorityTest>();
        break;
    case 1418042656594:
        return std::make_shared<MasterPlanTestConditionPlanType>();
        break;
    case 1418042796751:
        return std::make_shared<PreConditionPlan>();
        break;
    case 1418042806575:
        return std::make_shared<RuntimeConditionPlan>();
        break;
    case 1418042819203:
        return std::make_shared<OtherPlan>();
        break;
    case 1418825395939:
        return std::make_shared<MasterSyncTransition>();
        break;
    case 1418902217839:
        return std::make_shared<RealMasterPlanForSyncTest>();
        break;
    case 1428508768572:
        return std::make_shared<BehaviourTriggerTestPlan>();
        break;
    case 1522377375148:
        return std::make_shared<BehaviorSuccessSpamMaster>();
        break;
    case 1529456584982:
        return std::make_shared<BackForth>();
        break;
    case 1530004915640:
        return std::make_shared<HandleFailExplicit>();
        break;
    case 1530004940652:
        return std::make_shared<HandleFailExplicitMaster>();
        break;
    case 1530069246103:
        return std::make_shared<FailsOnOne>();
        break;
    case 1588060981661:
        return std::make_shared<ConfigurationTestPlan>();
        break;
    case 1588061334567:
        return std::make_shared<ReadConfigurationPlan>();
        break;
    case 1588061801734:
        return std::make_shared<ReadConfInPlantype>();
        break;
    default:
        std::cerr << "PlanCreator: Unknown plan requested: " << planId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
