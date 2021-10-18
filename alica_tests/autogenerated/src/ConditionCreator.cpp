#include "ConditionCreator.h"
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
    case 1840401110297459509:
        return std::make_shared<PreCondition1840401110297459509>();
        break;
    case 4115970455290610262:
        return std::make_shared<PreCondition4115970455290610262>();
        break;
    case 4238964946542987247:
        return std::make_shared<PreCondition4238964946542987247>();
    case 1840401110297459509:
        return std::make_shared<PreCondition1840401110297459509>();
        break;
    default:
        std::cerr << "ConditionCreator: Unknown condition id requested: " << conditionConfId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
