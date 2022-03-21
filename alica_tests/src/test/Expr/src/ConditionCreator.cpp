#include "ConditionCreator.h"
#include "AdjacentSuccessMasterPlan3254486013443203397.h"
#include "AdjacentSuccessSubPlan1682631238618360548.h"
#include "AssignPayload3826644292150922713.h"
#include "AttackPlan1402488634525.h"
#include "Authority/AuthorityTest1414403413451.h"
#include "Authority/AuthorityTestMaster1414403396328.h"
#include "BackForth1529456584982.h"
#include "BehaviorSuccessSpamMaster1522377375148.h"
#include "Behaviour/AlwaysFail1532424188199.h"
#include "Behaviour/Attack1402488848841.h"
#include "Behaviour/AttackOpp1402489351885.h"
#include "Behaviour/BehAAA1629895901559.h"
#include "Behaviour/BehBAA1629895911592.h"
#include "Behaviour/ConstraintUsingBehaviour1414068597716.h"
#include "Behaviour/CountIndefinitely1529456643148.h"
#include "Behaviour/DefendMid1402488730695.h"
#include "Behaviour/EmptyBehaviour1625610857563.h"
#include "Behaviour/MidFieldStandard1402488696205.h"
#include "Behaviour/NotToTrigger1429017274116.h"
#include "Behaviour/ReadConfigurationBehaviour1588061129360.h"
#include "Behaviour/SuccessSpam1522377401286.h"
#include "Behaviour/Tackle1402488939130.h"
#include "Behaviour/TriggerA1428508297492.h"
#include "Behaviour/TriggerB1428508316905.h"
#include "Behaviour/TriggerC1428508355209.h"
#include "BehaviourTriggerTestPlan1428508768572.h"
#include "Configurations/ConfigurationTestPlan1588060981661.h"
#include "Configurations/ReadConfInPlantype1588061801734.h"
#include "Configurations/ReadConfigurationPlan1588061334567.h"
#include "ConstraintTestMaster1414068495566.h"
#include "ConstraintTestPlan1414068524245.h"
#include "Defend1402488893641.h"
#include "Drop3009473645416620380.h"
#include "DynamicTaskAssignmentTest2252865124432942907.h"
#include "DynamicTaskAssignmentTestMaster1602078208698393838.h"
#include "DynamicTaskBehavior4044546549214673470.h"
#include "DynamicTaskBehaviourLD19516698765703926.h"
#include "DynamicTaskLA3337489358878214836.h"
#include "DynamicTaskLB4316676367342780557.h"
#include "DynamicTaskLC2140075868731779222.h"
#include "DynamicTaskTogether1338298120374694644.h"
#include "EmptyPlan984284423749038756.h"
#include "ExecuteBehaviourInSubPlan3172561495666303184.h"
#include "FailsOnOne1530069246103.h"
#include "FreePayload422054015709952219.h"
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
#include "NavigateToDrop4459885370764933844.h"
#include "NavigateToPick4505472195947429717.h"
#include "OrderedSchedulingTestPlan1629895582410.h"
#include "OtherPlan1418042819203.h"
#include "Pick2580816776008671737.h"
#include "PickDrop725594143882346503.h"
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
#include "SerializationMasterPlan373109241446504968.h"
#include "SerializationSubPlanA1433931143598606082.h"
#include "SerializationSubPlanB230205985761632608.h"
#include "SerializationSubPlanC2359124678252958039.h"
#include "SerializationSubPlanD1781630225028158279.h"
#include "SimpleTestPlan1412252439925.h"
#include "Tackle1402489318663.h"
#include "TaskInstantiationIntegrationTestMaster4603312216886200747.h"
#include "TestBehaviour55178365253414982.h"
#include "TestParameterPassing1692837668719979457.h"
#include "TestParameterPassingBehaviour831400441334251602.h"
#include "TestParameterPassingMaster1179066429431332055.h"
#include "TestTracingMasterPlan691392966514374878.h"
#include "TestTracingSubPlan1482512794732634139.h"

namespace alica
{

ConditionCreator::ConditionCreator() {}
ConditionCreator::~ConditionCreator() {}

std::shared_ptr<BasicCondition> ConditionCreator::createConditions(int64_t conditionConfId)
{
    switch (conditionConfId) {
    case 1402488519140:
        return std::make_shared<PreCondition1402488519140>();
        break;
    case 1402488520968:
        return std::make_shared<PreCondition1402488520968>();
        break;
    case 1402488558741:
        return std::make_shared<PreCondition1402488558741>();
        break;
    case 1402488991641:
        return std::make_shared<PreCondition1402488991641>();
        break;
    case 1402488993122:
        return std::make_shared<PreCondition1402488993122>();
        break;
    case 1402489065962:
        return std::make_shared<PreCondition1402489065962>();
        break;
    case 1402489073613:
        return std::make_shared<PreCondition1402489073613>();
        break;
    case 1402489131988:
        return std::make_shared<PreCondition1402489131988>();
        break;
    case 1402489174338:
        return std::make_shared<PreCondition1402489174338>();
        break;
    case 1402489206278:
        return std::make_shared<PreCondition1402489206278>();
        break;
    case 1402489218027:
        return std::make_shared<PreCondition1402489218027>();
        break;
    case 1402489258509:
        return std::make_shared<PreCondition1402489258509>();
        break;
    case 1402489260911:
        return std::make_shared<RunTimeCondition1402489260911>();
        break;
    case 1402489278408:
        return std::make_shared<PreCondition1402489278408>();
        break;
    case 1402489460549:
        return std::make_shared<PreCondition1402489460549>();
        break;
    case 1402489462088:
        return std::make_shared<PreCondition1402489462088>();
        break;
    case 1402489620773:
        return std::make_shared<PostCondition1402489620773>();
        break;
    case 1402500844446:
        return std::make_shared<PreCondition1402500844446>();
        break;
    case 1403773741874:
        return std::make_shared<RunTimeCondition1403773741874>();
        break;
    case 1409218319990:
        return std::make_shared<PreCondition1409218319990>();
        break;
    case 1412761926856:
        return std::make_shared<PreCondition1412761926856>();
        break;
    case 1412781693884:
        return std::make_shared<RunTimeCondition1412781693884>();
        break;
    case 1412781707952:
        return std::make_shared<PreCondition1412781707952>();
        break;
    case 1413201052549:
        return std::make_shared<PreCondition1413201052549>();
        break;
    case 1413201227586:
        return std::make_shared<PreCondition1413201227586>();
        break;
    case 1413201367990:
        return std::make_shared<PreCondition1413201367990>();
        break;
    case 1413201370590:
        return std::make_shared<PreCondition1413201370590>();
        break;
    case 1413201389955:
        return std::make_shared<PreCondition1413201389955>();
        break;
    case 1414068566297:
        return std::make_shared<RunTimeCondition1414068566297>();
        break;
    case 1414403842622:
        return std::make_shared<PreCondition1414403842622>();
        break;
    case 1418042683692:
        return std::make_shared<PreCondition1418042683692>();
        break;
    case 1418042929966:
        return std::make_shared<PreCondition1418042929966>();
        break;
    case 1418042967134:
        return std::make_shared<RunTimeCondition1418042967134>();
        break;
    case 1418825427317:
        return std::make_shared<PreCondition1418825427317>();
        break;
    case 1418825428924:
        return std::make_shared<PreCondition1418825428924>();
        break;
    case 1429017236633:
        return std::make_shared<PreCondition1429017236633>();
        break;
    case 1522377944921:
        return std::make_shared<PreCondition1522377944921>();
        break;
    case 1522377946607:
        return std::make_shared<PreCondition1522377946607>();
        break;
    case 1529456610697:
        return std::make_shared<PreCondition1529456610697>();
        break;
    case 1529456611916:
        return std::make_shared<PreCondition1529456611916>();
        break;
    case 1530004993493:
        return std::make_shared<PreCondition1530004993493>();
        break;
    case 1530004994611:
        return std::make_shared<PreCondition1530004994611>();
        break;
    case 1530069251117:
        return std::make_shared<RunTimeCondition1530069251117>();
        break;
    case 1532424093178:
        return std::make_shared<PreCondition1532424093178>();
        break;
    case 1532424113475:
        return std::make_shared<PreCondition1532424113475>();
        break;
    case 1588069612661:
        return std::make_shared<PreCondition1588069612661>();
        break;
    case 1588069615553:
        return std::make_shared<PreCondition1588069615553>();
        break;
    case 1588246141557:
        return std::make_shared<PreCondition1588246141557>();
        break;
    case 1588246144841:
        return std::make_shared<PreCondition1588246144841>();
        break;
    case 1588253347213:
        return std::make_shared<PreCondition1588253347213>();
        break;
    case 1613530643882:
        return std::make_shared<PreCondition1613530643882>();
        break;
    case 1614960055821:
        return std::make_shared<PreCondition1614960055821>();
        break;
    case 1614960063843:
        return std::make_shared<PreCondition1614960063843>();
        break;
    case 1614964566531:
        return std::make_shared<PreCondition1614964566531>();
        break;
    case 1614964572495:
        return std::make_shared<PreCondition1614964572495>();
        break;
    case 1614964575553:
        return std::make_shared<PreCondition1614964575553>();
        break;
    case 1614964578016:
        return std::make_shared<PreCondition1614964578016>();
        break;
    case 1615797316171:
        return std::make_shared<PreCondition1615797316171>();
        break;
    case 1615797327077:
        return std::make_shared<PreCondition1615797327077>();
        break;
    case 1615797365364:
        return std::make_shared<PreCondition1615797365364>();
        break;
    case 1629895598471:
        return std::make_shared<PreCondition1629895598471>();
        break;
    case 1629895607018:
        return std::make_shared<PreCondition1629895607018>();
        break;
    case 1629895758612:
        return std::make_shared<PreCondition1629895758612>();
        break;
    case 1629895768182:
        return std::make_shared<PreCondition1629895768182>();
        break;
    case 32970225314063392:
        return std::make_shared<PreCondition32970225314063392>();
        break;
    case 61978004585920576:
        return std::make_shared<PreCondition61978004585920576>();
        break;
    case 68542020926196536:
        return std::make_shared<PreCondition68542020926196536>();
        break;
    case 289358204208851392:
        return std::make_shared<PreCondition289358204208851392>();
        break;
    case 597347780541336226:
        return std::make_shared<PreCondition597347780541336226>();
        break;
    case 807250359520655888:
        return std::make_shared<PreCondition807250359520655888>();
        break;
    case 1067314038887345208:
        return std::make_shared<PreCondition1067314038887345208>();
        break;
    case 1078898265232036813:
        return std::make_shared<PreCondition1078898265232036813>();
        break;
    case 1693256954385338259:
        return std::make_shared<PreCondition1693256954385338259>();
        break;
    case 1840401110297459509:
        return std::make_shared<PreCondition1840401110297459509>();
        break;
    case 1943478533524176732:
        return std::make_shared<PreCondition1943478533524176732>();
        break;
    case 1971173312201839855:
        return std::make_shared<PreCondition1971173312201839855>();
        break;
    case 2132248203469102498:
        return std::make_shared<PreCondition2132248203469102498>();
        break;
    case 2187308102082241829:
        return std::make_shared<PreCondition2187308102082241829>();
        break;
    case 2915681556800498724:
        return std::make_shared<PreCondition2915681556800498724>();
        break;
    case 3126176581533900616:
        return std::make_shared<PreCondition3126176581533900616>();
        break;
    case 3213510506830850443:
        return std::make_shared<PreCondition3213510506830850443>();
        break;
    case 3461968191733792853:
        return std::make_shared<PreCondition3461968191733792853>();
        break;
    case 3691801807787093963:
        return std::make_shared<PreCondition3691801807787093963>();
        break;
    case 3932287302905544988:
        return std::make_shared<PreCondition3932287302905544988>();
        break;
    case 3953991713597643491:
        return std::make_shared<PreCondition3953991713597643491>();
        break;
    case 4115970455290610262:
        return std::make_shared<PreCondition4115970455290610262>();
        break;
    case 4165333637052704488:
        return std::make_shared<PreCondition4165333637052704488>();
        break;
    case 4238964946542987247:
        return std::make_shared<PreCondition4238964946542987247>();
        break;
    case 4344644064496100420:
        return std::make_shared<PreCondition4344644064496100420>();
        break;
    case 4496654201854254411:
        return std::make_shared<PreCondition4496654201854254411>();
        break;
    default:
        std::cerr << "ConditionCreator: Unknown condition id requested: " << conditionConfId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
