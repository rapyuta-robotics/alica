#include <iostream>
#include <test/AdjacentSuccessMasterPlan3254486013443203397.h>
#include <test/AdjacentSuccessSubPlan1682631238618360548.h>
#include <test/AttackPlan1402488634525.h>
#include <test/Authority/AuthorityTest1414403413451.h>
#include <test/Authority/AuthorityTestMaster1414403396328.h>
#include <test/BackForth1529456584982.h>
#include <test/BehaviorSuccessSpamMaster1522377375148.h>
#include <test/BehaviourTriggerTestPlan1428508768572.h>
#include <test/Configurations/ConfigurationTestPlan1588060981661.h>
#include <test/Configurations/ReadConfInPlantype1588061801734.h>
#include <test/Configurations/ReadConfigurationPlan1588061334567.h>
#include <test/ConstraintTestMaster1414068495566.h>
#include <test/ConstraintTestPlan1414068524245.h>
#include <test/Defend1402488893641.h>
#include <test/ExecuteBehaviourInSubPlan3172561495666303184.h>
#include <test/FailsOnOne1530069246103.h>
#include <test/FailureHandlingMaster4150733089768927549.h>
#include <test/FailurePlan631515556091266493.h>
#include <test/FrequencyTestPlan1626848999740.h>
#include <test/GoalPlan1402488870347.h>
#include <test/HandleFailExplicit1530004915640.h>
#include <test/HandleFailExplicitMaster1530004940652.h>
#include <test/MasterPlan1402488437260.h>
#include <test/MasterPlanTaskAssignment1407152758497.h>
#include <test/MasterPlanTestConditionPlanType1418042656594.h>
#include <test/MasterSyncTransition1418825395939.h>
#include <test/MidFieldPlayPlan1402488770050.h>
#include <test/MultiAgentTestMaster1413200842973.h>
#include <test/MultiAgentTestPlan1413200862180.h>
#include <test/OrderedSchedulingTestPlan1629895582410.h>
#include <test/OtherPlan1418042819203.h>
#include <test/PlanA1629895837159.h>
#include <test/PlanAA1629895864090.h>
#include <test/PlanB1629895853508.h>
#include <test/PlanBA1629895873188.h>
#include <test/PlanFive1407153703092.h>
#include <test/PlanFour1407153683051.h>
#include <test/PlanOne1407153611768.h>
#include <test/PlanThree1407153663917.h>
#include <test/PlanTwo1407153645238.h>
#include <test/PreConditionPlan1418042796751.h>
#include <test/RealMasterPlanForSyncTest1418902217839.h>
#include <test/RuntimeConditionPlan1418042806575.h>
#include <test/SchedulingTestMasterPlan1613378382024.h>
#include <test/SchedulingTestPlan11613378406860.h>
#include <test/SchedulingTestPlan21613378423610.h>
#include <test/SchedulingTestPlan31613378433623.h>
#include <test/SchedulingTestSequencePlan11614963946725.h>
#include <test/SchedulingTestSequenceSubPlan11614964379654.h>
#include <test/SchedulingTestSequenceSubPlan21614964444419.h>
#include <test/SchedulingTestSequenceSubPlan31614964478264.h>
#include <test/SimpleTestPlan1412252439925.h>
#include <test/Tackle1402489318663.h>
#include <test/TestInheritBlackboard1692837668719979400.h>
#include <test/TestInheritBlackboardMaster1179066429431332056.h>
#include <test/TestParameterPassing1692837668719979457.h>
#include <test/TestParameterPassingMaster1179066429431332055.h>
#include <test/TestTracingMasterPlan691392966514374878.h>
#include <test/TestTracingSubPlan1482512794732634139.h>
#include <test/UtilityFunctionCreator.h>

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
    case 631515556091266493:
        return std::make_shared<UtilityFunction631515556091266493>();
        break;
    case 691392966514374878:
        return std::make_shared<UtilityFunction691392966514374878>();
        break;
    case 1179066429431332055:
        return std::make_shared<UtilityFunction1179066429431332055>();
        break;
    case 1179066429431332056:
        return std::make_shared<UtilityFunction1179066429431332056>();
        break;
    case 1482512794732634139:
        return std::make_shared<UtilityFunction1482512794732634139>();
        break;
    case 1682631238618360548:
        return std::make_shared<UtilityFunction1682631238618360548>();
        break;
    case 1692837668719979400:
        return std::make_shared<UtilityFunction1692837668719979400>();
        break;
    case 1692837668719979457:
        return std::make_shared<UtilityFunction1692837668719979457>();
        break;
    case 3172561495666303184:
        return std::make_shared<UtilityFunction3172561495666303184>();
        break;
    case 3254486013443203397:
        return std::make_shared<UtilityFunction3254486013443203397>();
        break;
    case 4150733089768927549:
        return std::make_shared<UtilityFunction4150733089768927549>();
        break;
    default:
        std::cerr << "UtilityFunctionCreator: Unknown utility requested: " << utilityfunctionConfId << std::endl;
        throw new std::exception();
        break;
    }
}

} // namespace alica
