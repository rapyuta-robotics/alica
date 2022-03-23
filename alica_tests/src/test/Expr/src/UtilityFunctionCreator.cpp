#include "UtilityFunctionCreator.h"
#include "AdjacentSuccessMasterPlan3254486013443203397.h"
#include "AdjacentSuccessSubPlan1682631238618360548.h"
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
#include "DynamicTaskAssignmentTest2252865124432942907.h"
#include "DynamicTaskAssignmentTestMaster1602078208698393838.h"
#include "DynamicTaskLA3337489358878214836.h"
#include "DynamicTaskLB4316676367342780557.h"
#include "DynamicTaskLC2140075868731779222.h"
#include "DynamicTaskTogether1338298120374694644.h"
#include "EmptyPlan984284423749038756.h"
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
#include "MovePayload725594143882346503.h"
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
#include "SerializationMasterPlan373109241446504968.h"
#include "SerializationSubPlanA1433931143598606082.h"
#include "SerializationSubPlanB230205985761632608.h"
#include "SerializationSubPlanC2359124678252958039.h"
#include "SerializationSubPlanD1781630225028158279.h"
#include "SimpleTestPlan1412252439925.h"
#include "Tackle1402489318663.h"
#include "TaskInstantiationIntegrationTestMaster4603312216886200747.h"
#include "TestParameterPassing1692837668719979457.h"
#include "TestParameterPassingMaster1179066429431332055.h"
#include "TestTracingMasterPlan691392966514374878.h"
#include "TestTracingSubPlan1482512794732634139.h"
#include <iostream>

namespace alica
{

UtilityFunctionCreator::~UtilityFunctionCreator() {}

UtilityFunctionCreator::UtilityFunctionCreator() {}

std::shared_ptr<BasicUtilityFunction> UtilityFunctionCreator::createUtility(int64_t utilityfunctionConfId)
{
    switch (utilityfunctionConfId) {
    case 1402488437260:
        return std::make_shared<UtilityFunction1402488437260>();
        break;
    case 1402488634525:
        return std::make_shared<UtilityFunction1402488634525>();
        break;
    case 1402488770050:
        return std::make_shared<UtilityFunction1402488770050>();
        break;
    case 1402488870347:
        return std::make_shared<UtilityFunction1402488870347>();
        break;
    case 1402488893641:
        return std::make_shared<UtilityFunction1402488893641>();
        break;
    case 1402489318663:
        return std::make_shared<UtilityFunction1402489318663>();
        break;
    case 1407152758497:
        return std::make_shared<UtilityFunction1407152758497>();
        break;
    case 1407153611768:
        return std::make_shared<UtilityFunction1407153611768>();
        break;
    case 1407153645238:
        return std::make_shared<UtilityFunction1407153645238>();
        break;
    case 1407153663917:
        return std::make_shared<UtilityFunction1407153663917>();
        break;
    case 1407153683051:
        return std::make_shared<UtilityFunction1407153683051>();
        break;
    case 1407153703092:
        return std::make_shared<UtilityFunction1407153703092>();
        break;
    case 1412252439925:
        return std::make_shared<UtilityFunction1412252439925>();
        break;
    case 1413200842973:
        return std::make_shared<UtilityFunction1413200842973>();
        break;
    case 1413200862180:
        return std::make_shared<UtilityFunction1413200862180>();
        break;
    case 1414068495566:
        return std::make_shared<UtilityFunction1414068495566>();
        break;
    case 1414068524245:
        return std::make_shared<UtilityFunction1414068524245>();
        break;
    case 1414403396328:
        return std::make_shared<UtilityFunction1414403396328>();
        break;
    case 1414403413451:
        return std::make_shared<UtilityFunction1414403413451>();
        break;
    case 1418042656594:
        return std::make_shared<UtilityFunction1418042656594>();
        break;
    case 1418042796751:
        return std::make_shared<UtilityFunction1418042796751>();
        break;
    case 1418042806575:
        return std::make_shared<UtilityFunction1418042806575>();
        break;
    case 1418042819203:
        return std::make_shared<UtilityFunction1418042819203>();
        break;
    case 1418825395939:
        return std::make_shared<UtilityFunction1418825395939>();
        break;
    case 1418902217839:
        return std::make_shared<UtilityFunction1418902217839>();
        break;
    case 1428508768572:
        return std::make_shared<UtilityFunction1428508768572>();
        break;
    case 1522377375148:
        return std::make_shared<UtilityFunction1522377375148>();
        break;
    case 1529456584982:
        return std::make_shared<UtilityFunction1529456584982>();
        break;
    case 1530004915640:
        return std::make_shared<UtilityFunction1530004915640>();
        break;
    case 1530004940652:
        return std::make_shared<UtilityFunction1530004940652>();
        break;
    case 1530069246103:
        return std::make_shared<UtilityFunction1530069246103>();
        break;
    case 1588060981661:
        return std::make_shared<UtilityFunction1588060981661>();
        break;
    case 1588061334567:
        return std::make_shared<UtilityFunction1588061334567>();
        break;
    case 1588061801734:
        return std::make_shared<UtilityFunction1588061801734>();
        break;
    case 1613378382024:
        return std::make_shared<UtilityFunction1613378382024>();
        break;
    case 1613378406860:
        return std::make_shared<UtilityFunction1613378406860>();
        break;
    case 1613378423610:
        return std::make_shared<UtilityFunction1613378423610>();
        break;
    case 1613378433623:
        return std::make_shared<UtilityFunction1613378433623>();
        break;
    case 1614963946725:
        return std::make_shared<UtilityFunction1614963946725>();
        break;
    case 1614964379654:
        return std::make_shared<UtilityFunction1614964379654>();
        break;
    case 1614964444419:
        return std::make_shared<UtilityFunction1614964444419>();
        break;
    case 1614964478264:
        return std::make_shared<UtilityFunction1614964478264>();
        break;
    case 1626848999740:
        return std::make_shared<UtilityFunction1626848999740>();
        break;
    case 1629895582410:
        return std::make_shared<UtilityFunction1629895582410>();
        break;
    case 1629895837159:
        return std::make_shared<UtilityFunction1629895837159>();
        break;
    case 1629895853508:
        return std::make_shared<UtilityFunction1629895853508>();
        break;
    case 1629895864090:
        return std::make_shared<UtilityFunction1629895864090>();
        break;
    case 1629895873188:
        return std::make_shared<UtilityFunction1629895873188>();
        break;
    case 230205985761632608:
        return std::make_shared<UtilityFunction230205985761632608>();
        break;
    case 373109241446504968:
        return std::make_shared<UtilityFunction373109241446504968>();
        break;
    case 432995127772554364:
        return std::make_shared<UtilityFunction432995127772554364>();
        break;
    case 691392966514374878:
        return std::make_shared<UtilityFunction691392966514374878>();
        break;
    case 725594143882346503:
        return std::make_shared<UtilityFunction725594143882346503>();
        break;
    case 984284423749038756:
        return std::make_shared<UtilityFunction984284423749038756>();
        break;
    case 1179066429431332055:
        return std::make_shared<UtilityFunction1179066429431332055>();
        break;
    case 1338298120374694644:
        return std::make_shared<UtilityFunction1338298120374694644>();
        break;
    case 1433931143598606082:
        return std::make_shared<UtilityFunction1433931143598606082>();
        break;
    case 1482512794732634139:
        return std::make_shared<UtilityFunction1482512794732634139>();
        break;
    case 1602078208698393838:
        return std::make_shared<UtilityFunction1602078208698393838>();
        break;
    case 1682631238618360548:
        return std::make_shared<UtilityFunction1682631238618360548>();
        break;
    case 1692837668719979457:
        return std::make_shared<UtilityFunction1692837668719979457>();
        break;
    case 1781630225028158279:
        return std::make_shared<UtilityFunction1781630225028158279>();
        break;
    case 1964838032551226161:
        return std::make_shared<UtilityFunction1964838032551226161>();
        break;
    case 2140075868731779222:
        return std::make_shared<UtilityFunction2140075868731779222>();
        break;
    case 2252865124432942907:
        return std::make_shared<UtilityFunction2252865124432942907>();
        break;
    case 2359124678252958039:
        return std::make_shared<UtilityFunction2359124678252958039>();
        break;
    case 3172561495666303184:
        return std::make_shared<UtilityFunction3172561495666303184>();
        break;
    case 3254486013443203397:
        return std::make_shared<UtilityFunction3254486013443203397>();
        break;
    case 3337489358878214836:
        return std::make_shared<UtilityFunction3337489358878214836>();
        break;
    case 4316676367342780557:
        return std::make_shared<UtilityFunction4316676367342780557>();
        break;
    case 4603312216886200747:
        return std::make_shared<UtilityFunction4603312216886200747>();
        break;
    default:
        std::cerr << "UtilityFunctionCreator: Unknown utility requested: " << utilityfunctionConfId << std::endl;
        throw new std::exception();
        break;
    }
}

} // namespace alica
