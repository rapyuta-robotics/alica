#include "engine/BasicPlan.h"
#include <alica_tests/AdjacentSuccessMasterPlan3254486013443203397.h>
#include <alica_tests/AdjacentSuccessSubPlan1682631238618360548.h>
#include <alica_tests/AttackPlan1402488634525.h>
#include <alica_tests/Authority/AuthorityTest1414403413451.h>
#include <alica_tests/Authority/AuthorityTestMaster1414403396328.h>
#include <alica_tests/BackForth1529456584982.h>
#include <alica_tests/BehSuccessTestPlan2189867578804904568.h>
#include <alica_tests/BehaviourTriggerTestPlan1428508768572.h>
#include <alica_tests/Configurations/ConfigurationTestPlan1588060981661.h>
#include <alica_tests/Configurations/ReadConfInPlantype1588061801734.h>
#include <alica_tests/Configurations/ReadConfigurationPlan1588061334567.h>
#include <alica_tests/ConstraintTestMaster1414068495566.h>
#include <alica_tests/ConstraintTestPlan1414068524245.h>
#include <alica_tests/Defend1402488893641.h>
#include <alica_tests/ExecuteBehaviourInSubPlan3172561495666303184.h>
#include <alica_tests/FailsOnOne1530069246103.h>
#include <alica_tests/FailureHandlingMaster4150733089768927549.h>
#include <alica_tests/FailurePlan631515556091266493.h>
#include <alica_tests/FrequencyTestPlan1626848999740.h>
#include <alica_tests/GoalPlan1402488870347.h>
#include <alica_tests/HandleFailExplicit1530004915640.h>
#include <alica_tests/HandleFailExplicitMaster1530004940652.h>
#include <alica_tests/MasterPlan1402488437260.h>
#include <alica_tests/MasterPlanTaskAssignment1407152758497.h>
#include <alica_tests/MasterPlanTestConditionPlanType1418042656594.h>
#include <alica_tests/MasterSyncTransition1418825395939.h>
#include <alica_tests/MidFieldPlayPlan1402488770050.h>
#include <alica_tests/MultiAgentTestMaster1413200842973.h>
#include <alica_tests/MultiAgentTestPlan1413200862180.h>
#include <alica_tests/MultiPlanInstanceSuccessTestPlan3392981108193862307.h>
#include <alica_tests/OrderedSchedulingTestPlan1629895582410.h>
#include <alica_tests/OtherPlan1418042819203.h>
#include <alica_tests/ParallelSuccessOnCondPlan3288843407985944525.h>
#include <alica_tests/PlanA1629895837159.h>
#include <alica_tests/PlanAA1629895864090.h>
#include <alica_tests/PlanB1629895853508.h>
#include <alica_tests/PlanBA1629895873188.h>
#include <alica_tests/PlanCreator.h>
#include <alica_tests/PlanFive1407153703092.h>
#include <alica_tests/PlanFour1407153683051.h>
#include <alica_tests/PlanIsSuccess1522377375148.h>
#include <alica_tests/PlanOne1407153611768.h>
#include <alica_tests/PlanSuccessTestPlan3870436056558842479.h>
#include <alica_tests/PlanThree1407153663917.h>
#include <alica_tests/PlanTwo1407153645238.h>
#include <alica_tests/PreConditionPlan1418042796751.h>
#include <alica_tests/RealMasterPlanForSyncTest1418902217839.h>
#include <alica_tests/RunBehaviourInSimplePlan2504351804499332310.h>
#include <alica_tests/RuntimeConditionPlan1418042806575.h>
#include <alica_tests/SchedulingTestMasterPlan1613378382024.h>
#include <alica_tests/SchedulingTestPlan11613378406860.h>
#include <alica_tests/SchedulingTestPlan21613378423610.h>
#include <alica_tests/SchedulingTestPlan31613378433623.h>
#include <alica_tests/SchedulingTestSequencePlan11614963946725.h>
#include <alica_tests/SchedulingTestSequenceSubPlan11614964379654.h>
#include <alica_tests/SchedulingTestSequenceSubPlan21614964444419.h>
#include <alica_tests/SchedulingTestSequenceSubPlan31614964478264.h>
#include <alica_tests/SimpleTestPlan1412252439925.h>
#include <alica_tests/SuccessOnCondPlan3153116020668535682.h>
#include <alica_tests/SuccessOnCondWrapperAPlan673160616613514188.h>
#include <alica_tests/SuccessOnCondWrapperBPlan2869465844414224272.h>
#include <alica_tests/SuccessOnInitPlan1863216812678266511.h>
#include <alica_tests/Tackle1402489318663.h>
#include <alica_tests/TestInheritBlackboard1692837668719979400.h>
#include <alica_tests/TestInheritBlackboardMaster1179066429431332056.h>
#include <alica_tests/TestMasterPlan2521443078354411465.h>
#include <alica_tests/TestParameterPassing1692837668719979457.h>
#include <alica_tests/TestParameterPassingMaster1179066429431332055.h>
#include <alica_tests/TestTracingMasterPlan691392966514374878.h>
#include <alica_tests/TestTracingSubPlan1482512794732634139.h>

namespace alica
{

PlanCreator::PlanCreator() {}

PlanCreator::~PlanCreator() {}

std::unique_ptr<BasicPlan> PlanCreator::createPlan(int64_t planId, PlanContext& context)
{
    switch (planId) {
    case 1402488437260:
        return std::make_unique<MasterPlan1402488437260>(context);
        break;
    case 1402488634525:
        return std::make_unique<AttackPlan1402488634525>(context);
        break;
    case 1402488770050:
        return std::make_unique<MidFieldPlayPlan1402488770050>(context);
        break;
    case 1402488870347:
        return std::make_unique<GoalPlan1402488870347>(context);
        break;
    case 1402488893641:
        return std::make_unique<Defend1402488893641>(context);
        break;
    case 1402489318663:
        return std::make_unique<Tackle1402489318663>(context);
        break;
    case 1407152758497:
        return std::make_unique<MasterPlanTaskAssignment1407152758497>(context);
        break;
    case 1407153611768:
        return std::make_unique<PlanOne1407153611768>(context);
        break;
    case 1407153645238:
        return std::make_unique<PlanTwo1407153645238>(context);
        break;
    case 1407153663917:
        return std::make_unique<PlanThree1407153663917>(context);
        break;
    case 1407153683051:
        return std::make_unique<PlanFour1407153683051>(context);
        break;
    case 1407153703092:
        return std::make_unique<PlanFive1407153703092>(context);
        break;
    case 1412252439925:
        return std::make_unique<SimpleTestPlan1412252439925>(context);
        break;
    case 1413200842973:
        return std::make_unique<MultiAgentTestMaster1413200842973>(context);
        break;
    case 1413200862180:
        return std::make_unique<MultiAgentTestPlan1413200862180>(context);
        break;
    case 1414068495566:
        return std::make_unique<ConstraintTestMaster1414068495566>(context);
        break;
    case 1414068524245:
        return std::make_unique<ConstraintTestPlan1414068524245>(context);
        break;
    case 1414403396328:
        return std::make_unique<AuthorityTestMaster1414403396328>(context);
        break;
    case 1414403413451:
        return std::make_unique<AuthorityTest1414403413451>(context);
        break;
    case 1418042656594:
        return std::make_unique<MasterPlanTestConditionPlanType1418042656594>(context);
        break;
    case 1418042796751:
        return std::make_unique<PreConditionPlan1418042796751>(context);
        break;
    case 1418042806575:
        return std::make_unique<RuntimeConditionPlan1418042806575>(context);
        break;
    case 1418042819203:
        return std::make_unique<OtherPlan1418042819203>(context);
        break;
    case 1418825395939:
        return std::make_unique<MasterSyncTransition1418825395939>(context);
        break;
    case 1418902217839:
        return std::make_unique<RealMasterPlanForSyncTest1418902217839>(context);
        break;
    case 1428508768572:
        return std::make_unique<BehaviourTriggerTestPlan1428508768572>(context);
        break;
    case 1522377375148:
        return std::make_unique<PlanIsSuccess1522377375148>(context);
        break;
    case 1529456584982:
        return std::make_unique<BackForth1529456584982>(context);
        break;
    case 1530004915640:
        return std::make_unique<HandleFailExplicit1530004915640>(context);
        break;
    case 1530004940652:
        return std::make_unique<HandleFailExplicitMaster1530004940652>(context);
        break;
    case 1530069246103:
        return std::make_unique<FailsOnOne1530069246103>(context);
        break;
    case 1588060981661:
        return std::make_unique<ConfigurationTestPlan1588060981661>(context);
        break;
    case 1588061334567:
        return std::make_unique<ReadConfigurationPlan1588061334567>(context);
        break;
    case 1588061801734:
        return std::make_unique<ReadConfInPlantype1588061801734>(context);
        break;
    case 1613378382024:
        return std::make_unique<SchedulingTestMasterPlan1613378382024>(context);
        break;
    case 1613378406860:
        return std::make_unique<SchedulingTestPlan11613378406860>(context);
        break;
    case 1613378423610:
        return std::make_unique<SchedulingTestPlan21613378423610>(context);
        break;
    case 1613378433623:
        return std::make_unique<SchedulingTestPlan31613378433623>(context);
        break;
    case 1614963946725:
        return std::make_unique<SchedulingTestSequencePlan11614963946725>(context);
        break;
    case 1614964379654:
        return std::make_unique<SchedulingTestSequenceSubPlan11614964379654>(context);
        break;
    case 1614964444419:
        return std::make_unique<SchedulingTestSequenceSubPlan21614964444419>(context);
        break;
    case 1614964478264:
        return std::make_unique<SchedulingTestSequenceSubPlan31614964478264>(context);
        break;
    case 1626848999740:
        return std::make_unique<FrequencyTestPlan1626848999740>(context);
        break;
    case 1629895582410:
        return std::make_unique<OrderedSchedulingTestPlan1629895582410>(context);
        break;
    case 1629895837159:
        return std::make_unique<PlanA1629895837159>(context);
        break;
    case 1629895853508:
        return std::make_unique<PlanB1629895853508>(context);
        break;
    case 1629895864090:
        return std::make_unique<PlanAA1629895864090>(context);
        break;
    case 1629895873188:
        return std::make_unique<PlanBA1629895873188>(context);
        break;
    case 631515556091266493:
        return std::make_unique<FailurePlan631515556091266493>(context);
        break;
    case 673160616613514188:
        return std::make_unique<SuccessOnCondWrapperAPlan673160616613514188>(context);
        break;
    case 691392966514374878:
        return std::make_unique<TestTracingMasterPlan691392966514374878>(context);
        break;
    case 1179066429431332055:
        return std::make_unique<TestParameterPassingMaster1179066429431332055>(context);
        break;
    case 1179066429431332056:
        return std::make_unique<TestInheritBlackboardMaster1179066429431332056>(context);
        break;
    case 1482512794732634139:
        return std::make_unique<TestTracingSubPlan1482512794732634139>(context);
        break;
    case 1682631238618360548:
        return std::make_unique<AdjacentSuccessSubPlan1682631238618360548>(context);
        break;
    case 1692837668719979400:
        return std::make_unique<TestInheritBlackboard1692837668719979400>(context);
        break;
    case 1692837668719979457:
        return std::make_unique<TestParameterPassing1692837668719979457>(context);
        break;
    case 1863216812678266511:
        return std::make_unique<SuccessOnInitPlan1863216812678266511>(context);
        break;
    case 2189867578804904568:
        return std::make_unique<BehSuccessTestPlan2189867578804904568>(context);
        break;
    case 2504351804499332310:
        return std::make_unique<RunBehaviourInSimplePlan2504351804499332310>(context);
        break;
    case 2521443078354411465:
        return std::make_unique<TestMasterPlan2521443078354411465>(context);
        break;
    case 2869465844414224272:
        return std::make_unique<SuccessOnCondWrapperBPlan2869465844414224272>(context);
        break;
    case 3153116020668535682:
        return std::make_unique<SuccessOnCondPlan3153116020668535682>(context);
        break;
    case 3172561495666303184:
        return std::make_unique<ExecuteBehaviourInSubPlan3172561495666303184>(context);
        break;
    case 3254486013443203397:
        return std::make_unique<AdjacentSuccessMasterPlan3254486013443203397>(context);
        break;
    case 3288843407985944525:
        return std::make_unique<ParallelSuccessOnCondPlan3288843407985944525>(context);
        break;
    case 3392981108193862307:
        return std::make_unique<MultiPlanInstanceSuccessTestPlan3392981108193862307>(context);
        break;
    case 3870436056558842479:
        return std::make_unique<PlanSuccessTestPlan3870436056558842479>(context);
        break;
    case 4150733089768927549:
        return std::make_unique<FailureHandlingMaster4150733089768927549>(context);
        break;
    default:
        std::cerr << "PlanCreator: Unknown plan requested: " << planId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
