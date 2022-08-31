#include <alica_tests/AdjacentSuccessMasterPlan3254486013443203397.h>
#include <alica_tests/AdjacentSuccessSubPlan1682631238618360548.h>
#include <alica_tests/AttackPlan1402488634525.h>
#include <alica_tests/Authority/AuthorityTest1414403413451.h>
#include <alica_tests/Authority/AuthorityTestMaster1414403396328.h>
#include <alica_tests/BackForth1529456584982.h>
#include <alica_tests/BehaviorSuccessSpamMaster1522377375148.h>
#include <alica_tests/Behaviour/AlwaysFail1532424188199.h>
#include <alica_tests/Behaviour/Attack1402488848841.h>
#include <alica_tests/Behaviour/AttackOpp1402489351885.h>
#include <alica_tests/Behaviour/BehAAA1629895901559.h>
#include <alica_tests/Behaviour/BehBAA1629895911592.h>
#include <alica_tests/Behaviour/ConstraintUsingBehaviour1414068597716.h>
#include <alica_tests/Behaviour/CountIndefinitely1529456643148.h>
#include <alica_tests/Behaviour/DefendMid1402488730695.h>
#include <alica_tests/Behaviour/EmptyBehaviour1625610857563.h>
#include <alica_tests/Behaviour/MidFieldStandard1402488696205.h>
#include <alica_tests/Behaviour/NotToTrigger1429017274116.h>
#include <alica_tests/Behaviour/ReadConfigurationBehaviour1588061129360.h>
#include <alica_tests/Behaviour/SuccessSpam1522377401286.h>
#include <alica_tests/Behaviour/Tackle1402488939130.h>
#include <alica_tests/Behaviour/TestInheritBlackboardBehaviour831400441334251600.h>
#include <alica_tests/Behaviour/TestParameterPassingBehaviour831400441334251602.h>
#include <alica_tests/Behaviour/TriggerA1428508297492.h>
#include <alica_tests/Behaviour/TriggerB1428508316905.h>
#include <alica_tests/Behaviour/TriggerC1428508355209.h>
#include <alica_tests/BehaviourTriggerTestPlan1428508768572.h>
#include <alica_tests/ConditionCreator.h>
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
#include <alica_tests/OrderedSchedulingTestPlan1629895582410.h>
#include <alica_tests/OtherPlan1418042819203.h>
#include <alica_tests/PlanA1629895837159.h>
#include <alica_tests/PlanAA1629895864090.h>
#include <alica_tests/PlanB1629895853508.h>
#include <alica_tests/PlanBA1629895873188.h>
#include <alica_tests/PlanFive1407153703092.h>
#include <alica_tests/PlanFour1407153683051.h>
#include <alica_tests/PlanOne1407153611768.h>
#include <alica_tests/PlanThree1407153663917.h>
#include <alica_tests/PlanTwo1407153645238.h>
#include <alica_tests/PreConditionPlan1418042796751.h>
#include <alica_tests/RealMasterPlanForSyncTest1418902217839.h>
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
#include <alica_tests/Tackle1402489318663.h>
#include <alica_tests/TestBehaviour55178365253414982.h>
#include <alica_tests/TestInheritBlackboard1692837668719979400.h>
#include <alica_tests/TestInheritBlackboardMaster1179066429431332056.h>
#include <alica_tests/TestParameterPassing1692837668719979457.h>
#include <alica_tests/TestParameterPassingMaster1179066429431332055.h>
#include <alica_tests/TestTracingMasterPlan691392966514374878.h>
#include <alica_tests/TestTracingSubPlan1482512794732634139.h>

namespace alica
{

ConditionCreator::ConditionCreator() {}
ConditionCreator::~ConditionCreator() {}

std::shared_ptr<BasicCondition> ConditionCreator::createConditions(int64_t conditionConfId)
{
    switch (conditionConfId) {
    case 1402489131988:
        return std::make_shared<PreCondition1402489131988>();
        break;
    case 1402489260911:
        return std::make_shared<RunTimeCondition1402489260911>();
        break;
    case 1402489620773:
        return std::make_shared<PostCondition1402489620773>();
        break;
    case 1403773741874:
        return std::make_shared<RunTimeCondition1403773741874>();
        break;
    case 1412781693884:
        return std::make_shared<RunTimeCondition1412781693884>();
        break;
    case 1412781707952:
        return std::make_shared<PreCondition1412781707952>();
        break;
    case 1414068566297:
        return std::make_shared<RunTimeCondition1414068566297>();
        break;
    case 1418042929966:
        return std::make_shared<PreCondition1418042929966>();
        break;
    case 1418042967134:
        return std::make_shared<RunTimeCondition1418042967134>();
        break;
    case 1530069251117:
        return std::make_shared<RunTimeCondition1530069251117>();
        break;
    default:
        std::cerr << "ConditionCreator: Unknown condition id requested: " << conditionConfId << std::endl;
        throw new std::exception();
        break;
    }
}

std::shared_ptr<BasicCondition> ConditionCreator::createConditions(const ConditionContext& conditionContext)
{
    return nullptr;
}
} // namespace alica
