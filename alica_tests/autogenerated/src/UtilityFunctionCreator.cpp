#include "UtilityFunctionCreator.h"
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
#include "FailsOnOne1530069246103.h"
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
#include "OtherPlan1418042819203.h"
#include "PlanFive1407153703092.h"
#include "PlanFour1407153683051.h"
#include "PlanOne1407153611768.h"
#include "PlanThree1407153663917.h"
#include "PlanTwo1407153645238.h"
#include "PreConditionPlan1418042796751.h"
#include "RealMasterPlanForSyncTest1418902217839.h"
#include "RuntimeConditionPlan1418042806575.h"
#include "SimpleTestPlan1412252439925.h"
#include "Tackle1402489318663.h"
#include <iostream>

namespace alica
{

UtilityFunctionCreator::~UtilityFunctionCreator() {}

UtilityFunctionCreator::UtilityFunctionCreator() {}

std::shared_ptr<BasicUtilityFunction> UtilityFunctionCreator::createUtility(long utilityfunctionConfId)
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
    default:
        std::cerr << "UtilityFunctionCreator: Unknown utility requested: " << utilityfunctionConfId << std::endl;
        throw new std::exception();
        break;
    }
}

} // namespace alica
