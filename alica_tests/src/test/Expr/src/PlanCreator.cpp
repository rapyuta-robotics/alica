#include "PlanCreator.h"
#include "AttackPlan1402488634525.h"
#include "Authority/AuthorityTest1414403413451.h"
#include "Authority/AuthorityTestMaster1414403396328.h"
#include "BackForth1529456584982.h"
#include "BehaviorSuccessSpamMaster1522377375148.h"
#include "BehaviourTriggerTestPlan1428508768572.h"
#include "Configurations/ConfigurationTestPlan1588060981661.h"
#include "Configurations/ReadConfInPlantype1588061801734.h"
#include "Configurations/ReadConfigurationPlan1588061334567.h"
#include "ConstraintTestMaster1414068495566.h"
#include "ConstraintTestPlan1414068524245.h"
#include "Defend1402488893641.h"
#include "ExecuteBehaviourInSubPlan3172561495666303184.h"
#include "FailsOnOne1530069246103.h"
#include "FrequencyTestPlan1626848999740.h"
#include "GoalPlan1402488870347.h"
#include "HandleFailExplicit1530004915640.h"
#include "HandleFailExplicitMaster1530004940652.h"
#include "MasterPlan1402488437260.h"
#include "MasterPlanTaskAssignment1407152758497.h"
#include "MasterPlanTestConditionPlanType1418042656594.h"
#include "MasterSyncTransition1418825395939.h"
#include "MidFieldPlayPlan1402488770050.h"
#include "MultiAgentTestMaster1413200842973.h"
#include "MultiAgentTestPlan1413200862180.h"
#include "OrderedSchedulingTestPlan1629895582410.h"
#include "OtherPlan1418042819203.h"
#include "PlanA1629895837159.h"
#include "PlanAA1629895864090.h"
#include "PlanB1629895853508.h"
#include "PlanBA1629895873188.h"
#include "PlanFive1407153703092.h"
#include "PlanFour1407153683051.h"
#include "PlanOne1407153611768.h"
#include "PlanPoolTestMasterPlan1964838032551226161.h"
#include "PlanPoolTestSubPlan432995127772554364.h"
#include "PlanThree1407153663917.h"
#include "PlanTwo1407153645238.h"
#include "PreConditionPlan1418042796751.h"
#include "RealMasterPlanForSyncTest1418902217839.h"
#include "RuntimeConditionPlan1418042806575.h"
#include "SchedulingTestMasterPlan1613378382024.h"
#include "SchedulingTestPlan11613378406860.h"
#include "SchedulingTestPlan21613378423610.h"
#include "SchedulingTestPlan31613378433623.h"
#include "SchedulingTestSequencePlan11614963946725.h"
#include "SchedulingTestSequenceSubPlan11614964379654.h"
#include "SchedulingTestSequenceSubPlan21614964444419.h"
#include "SchedulingTestSequenceSubPlan31614964478264.h"
#include "SimpleTestPlan1412252439925.h"
#include "Tackle1402489318663.h"
#include "TestTracingMasterPlan691392966514374878.h"
#include "TestTracingSubPlan1482512794732634139.h"
#include "engine/BasicPlan.h"
#include "engine/IAlicaWorldModel.h"

namespace alica
{

PlanCreator::PlanCreator() {}

PlanCreator::~PlanCreator() {}

std::unique_ptr<BasicPlan> PlanCreator::createPlan(int64_t planId, IAlicaWorldModel* wm)
{
    switch (planId) {
    case 1402488437260:
        return std::make_unique<MasterPlan1402488437260>(wm);
        break;
    case 1402488634525:
        return std::make_unique<AttackPlan1402488634525>(wm);
        break;
    case 1402488770050:
        return std::make_unique<MidFieldPlayPlan1402488770050>(wm);
        break;
    case 1402488870347:
        return std::make_unique<GoalPlan1402488870347>(wm);
        break;
    case 1402488893641:
        return std::make_unique<Defend1402488893641>(wm);
        break;
    case 1402489318663:
        return std::make_unique<Tackle1402489318663>(wm);
        break;
    case 1407152758497:
        return std::make_unique<MasterPlanTaskAssignment1407152758497>(wm);
        break;
    case 1407153611768:
        return std::make_unique<PlanOne1407153611768>(wm);
        break;
    case 1407153645238:
        return std::make_unique<PlanTwo1407153645238>(wm);
        break;
    case 1407153663917:
        return std::make_unique<PlanThree1407153663917>(wm);
        break;
    case 1407153683051:
        return std::make_unique<PlanFour1407153683051>(wm);
        break;
    case 1407153703092:
        return std::make_unique<PlanFive1407153703092>(wm);
        break;
    case 1412252439925:
        return std::make_unique<SimpleTestPlan1412252439925>(wm);
        break;
    case 1413200842973:
        return std::make_unique<MultiAgentTestMaster1413200842973>(wm);
        break;
    case 1413200862180:
        return std::make_unique<MultiAgentTestPlan1413200862180>(wm);
        break;
    case 1414068495566:
        return std::make_unique<ConstraintTestMaster1414068495566>(wm);
        break;
    case 1414068524245:
        return std::make_unique<ConstraintTestPlan1414068524245>(wm);
        break;
    case 1414403396328:
        return std::make_unique<AuthorityTestMaster1414403396328>(wm);
        break;
    case 1414403413451:
        return std::make_unique<AuthorityTest1414403413451>(wm);
        break;
    case 1418042656594:
        return std::make_unique<MasterPlanTestConditionPlanType1418042656594>(wm);
        break;
    case 1418042796751:
        return std::make_unique<PreConditionPlan1418042796751>(wm);
        break;
    case 1418042806575:
        return std::make_unique<RuntimeConditionPlan1418042806575>(wm);
        break;
    case 1418042819203:
        return std::make_unique<OtherPlan1418042819203>(wm);
        break;
    case 1418825395939:
        return std::make_unique<MasterSyncTransition1418825395939>(wm);
        break;
    case 1418902217839:
        return std::make_unique<RealMasterPlanForSyncTest1418902217839>(wm);
        break;
    case 1428508768572:
        return std::make_unique<BehaviourTriggerTestPlan1428508768572>(wm);
        break;
    case 1522377375148:
        return std::make_unique<BehaviorSuccessSpamMaster1522377375148>(wm);
        break;
    case 1529456584982:
        return std::make_unique<BackForth1529456584982>(wm);
        break;
    case 1530004915640:
        return std::make_unique<HandleFailExplicit1530004915640>(wm);
        break;
    case 1530004940652:
        return std::make_unique<HandleFailExplicitMaster1530004940652>(wm);
        break;
    case 1530069246103:
        return std::make_unique<FailsOnOne1530069246103>(wm);
        break;
    case 1588060981661:
        return std::make_unique<ConfigurationTestPlan1588060981661>(wm);
        break;
    case 1588061334567:
        return std::make_unique<ReadConfigurationPlan1588061334567>(wm);
        break;
    case 1588061801734:
        return std::make_unique<ReadConfInPlantype1588061801734>(wm);
        break;
    case 1613378382024:
        return std::make_unique<SchedulingTestMasterPlan1613378382024>(wm);
        break;
    case 1613378406860:
        return std::make_unique<SchedulingTestPlan11613378406860>(wm);
        break;
    case 1613378423610:
        return std::make_unique<SchedulingTestPlan21613378423610>(wm);
        break;
    case 1613378433623:
        return std::make_unique<SchedulingTestPlan31613378433623>(wm);
        break;
    case 1614963946725:
        return std::make_unique<SchedulingTestSequencePlan11614963946725>(wm);
        break;
    case 1614964379654:
        return std::make_unique<SchedulingTestSequenceSubPlan11614964379654>(wm);
        break;
    case 1614964444419:
        return std::make_unique<SchedulingTestSequenceSubPlan21614964444419>(wm);
        break;
    case 1614964478264:
        return std::make_unique<SchedulingTestSequenceSubPlan31614964478264>(wm);
        break;
    case 1626848999740:
        return std::make_unique<FrequencyTestPlan1626848999740>(wm);
        break;
    case 1629895582410:
        return std::make_unique<OrderedSchedulingTestPlan1629895582410>(wm);
        break;
    case 1629895837159:
        return std::make_unique<PlanA1629895837159>(wm);
        break;
    case 1629895853508:
        return std::make_unique<PlanB1629895853508>(wm);
        break;
    case 1629895864090:
        return std::make_unique<PlanAA1629895864090>(wm);
        break;
    case 1629895873188:
        return std::make_unique<PlanBA1629895873188>(wm);
        break;
    case 432995127772554364:
        return std::make_unique<PlanPoolTestSubPlan432995127772554364>(wm);
        break;
    case 691392966514374878:
        return std::make_unique<TestTracingMasterPlan691392966514374878>(wm);
        break;
    case 1482512794732634139:
        return std::make_unique<TestTracingSubPlan1482512794732634139>(wm);
        break;
    case 1964838032551226161:
        return std::make_unique<PlanPoolTestMasterPlan1964838032551226161>(wm);
        break;
    case 3172561495666303184:
        return std::make_unique<ExecuteBehaviourInSubPlan3172561495666303184>(wm);
        break;
    default:
        std::cerr << "PlanCreator: Unknown plan requested: " << planId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
