#include <alica_tests/AdjacentSuccessMasterPlan.h>
#include <alica_tests/AdjacentSuccessSubPlan.h>
#include <alica_tests/AttackPlan.h>
#include <alica_tests/Authority/AuthorityTest.h>
#include <alica_tests/Authority/AuthorityTestMaster.h>
#include <alica_tests/BackForth.h>
#include <alica_tests/BehSuccessTestPlan.h>
#include <alica_tests/BehaviorSuccessSpamMaster.h>
#include <alica_tests/BehaviourTriggerTestPlan.h>
#include <alica_tests/ConditionCreator.h>
#include <alica_tests/Configurations/ConfigurationTestPlan.h>
#include <alica_tests/Configurations/ReadConfInPlantype.h>
#include <alica_tests/Configurations/ReadConfigurationPlan.h>
#include <alica_tests/ConstraintTestMaster.h>
#include <alica_tests/ConstraintTestPlan.h>
#include <alica_tests/Defend.h>
#include <alica_tests/ExecuteBehaviourInSubPlan.h>
#include <alica_tests/FailsOnOne.h>
#include <alica_tests/FailureHandlingMaster.h>
#include <alica_tests/FailurePlan.h>
#include <alica_tests/FrequencyTestPlan.h>
#include <alica_tests/GoalPlan.h>
#include <alica_tests/HandleFailExplicit.h>
#include <alica_tests/HandleFailExplicitMaster.h>
#include <alica_tests/MasterPlan.h>
#include <alica_tests/MasterPlanTaskAssignment.h>
#include <alica_tests/MasterPlanTestConditionPlanType.h>
#include <alica_tests/MasterSyncTransition.h>
#include <alica_tests/MidFieldPlayPlan.h>
#include <alica_tests/MultiAgentTestMaster.h>
#include <alica_tests/MultiAgentTestPlan.h>
#include <alica_tests/MultiPlanInstanceSuccessTestPlan.h>
#include <alica_tests/OrderedSchedulingTestPlan.h>
#include <alica_tests/OtherPlan.h>
#include <alica_tests/ParallelSuccessOnCondPlan.h>
#include <alica_tests/PlanA.h>
#include <alica_tests/PlanAA.h>
#include <alica_tests/PlanB.h>
#include <alica_tests/PlanBA.h>
#include <alica_tests/PlanFive.h>
#include <alica_tests/PlanFour.h>
#include <alica_tests/PlanOne.h>
#include <alica_tests/PlanSuccessTestPlan.h>
#include <alica_tests/PlanThree.h>
#include <alica_tests/PlanTwo.h>
#include <alica_tests/PreConditionPlan.h>
#include <alica_tests/RealMasterPlanForSyncTest.h>
#include <alica_tests/RuntimeConditionPlan.h>
#include <alica_tests/SchedulingTestMasterPlan.h>
#include <alica_tests/SchedulingTestPlan1.h>
#include <alica_tests/SchedulingTestPlan2.h>
#include <alica_tests/SchedulingTestPlan3.h>
#include <alica_tests/SchedulingTestSequencePlan1.h>
#include <alica_tests/SchedulingTestSequenceSubPlan1.h>
#include <alica_tests/SchedulingTestSequenceSubPlan2.h>
#include <alica_tests/SchedulingTestSequenceSubPlan3.h>
#include <alica_tests/SimpleTestPlan.h>
#include <alica_tests/SuccessOnCondPlan.h>
#include <alica_tests/SuccessOnCondWrapperAPlan.h>
#include <alica_tests/SuccessOnCondWrapperBPlan.h>
#include <alica_tests/SuccessOnInitPlan.h>
#include <alica_tests/Tackle.h>
#include <alica_tests/TestInheritBlackboard.h>
#include <alica_tests/TestInheritBlackboardMaster.h>
#include <alica_tests/TestMasterPlan.h>
#include <alica_tests/TestParameterPassing.h>
#include <alica_tests/TestParameterPassingMaster.h>
#include <alica_tests/TestTracingMasterPlan.h>
#include <alica_tests/TestTracingSubPlan.h>

namespace alica
{

ConditionCreator::ConditionCreator() {}
ConditionCreator::~ConditionCreator() {}

std::shared_ptr<BasicCondition> ConditionCreator::createConditions(int64_t conditionConfId, ConditionContext& context)
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
} // namespace alica
