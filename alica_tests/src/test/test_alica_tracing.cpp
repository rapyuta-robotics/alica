#include "test_alica.h"

#include "Behaviour/Attack.h"
#include "Behaviour/MidFieldStandard.h"
#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "PlanCreator.h"
#include "UtilityFunctionCreator.h"

#include <alica_tests/CounterClass.h>
#include <alica_tests/DummyTestSummand.h>
#include <alica_tests/TestWorldModel.h>

#include <alica/test/Util.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/Assignment.h>
#include <engine/BasicBehaviour.h>
#include <engine/BehaviourPool.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaCommunication.h>
#include <engine/PlanBase.h>
#include <engine/PlanRepository.h>
#include <engine/TeamObserver.h>
#include <engine/model/Behaviour.h>
#include <engine/model/Plan.h>
#include <engine/model/RuntimeCondition.h>
#include <engine/model/State.h>
#include <engine/UtilityFunction.h>
#include <engine/teammanager/TeamManager.h>
#include <engine/allocationauthority/AllocationDifference.h>
#include <engine/allocationauthority/EntryPointRobotPair.h>
#include <engine/model/Task.h>
#include <engine/modelmanagement/factories/TaskFactory.h>
#include <communication/AlicaDummyCommunication.h>

#include <gtest/gtest.h>

namespace alica
{
namespace
{

class AlicaTracingTest : public AlicaTestTracingFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "TestTracingMasterPlan"; }
    bool stepEngine() const override { return false; }
};

class AlicaAuthorityTracingTest : public AlicaTestMultiAgentTracingFixture
{
protected:
    const int agentCount = 2;
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "AuthorityTestMaster"; }
    int getAgentCount() const override { return agentCount; }
    const char* getHostName(int agentNumber) const override
    {
        if (agentNumber) {
            return "hairy";
        } else {
            return "nase";
        }
    }
};

TEST_F(AlicaTracingTest, runTracing)
{
    ASSERT_NO_SIGNAL
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(200));

    alicaTests::TestWorldModel::getOne()->setPreCondition1840401110297459509(true);
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(200));

    EXPECT_EQ(alicaTests::TestWorldModel::getOne()->tracingParents["EmptyBehaviour"], "TestTracingSubPlan");
    EXPECT_EQ(alicaTests::TestWorldModel::getOne()->tracingParents["TestTracingSubPlan"], "TestTracingMasterPlan");
}

TEST_F(AlicaAuthorityTracingTest, taskAssignmentTracing)
{
    const Plan* plan = aes[0]->getPlanRepository().getPlans().find(1414403413451);
    ASSERT_NE(plan, nullptr) << "Plan 1414403413451 is unknown";
    ASSERT_NE(plan->getUtilityFunction(), nullptr) << "UtilityFunction is null!";
    auto uSummandAe = plan->getUtilityFunction()->getUtilSummands()[0].get();
    alica::DummyTestSummand* dbr = dynamic_cast<alica::DummyTestSummand*>(uSummandAe);
    dbr->robotId = acs[0]->getLocalAgentId();

    auto uSummandAe2 = aes[1]->getPlanRepository().getPlans().find(1414403413451)->getUtilityFunction()->getUtilSummands()[0].get();
    alica::DummyTestSummand* dbr2 = dynamic_cast<alica::DummyTestSummand*>(uSummandAe2);
    dbr2->robotId = acs[1]->getLocalAgentId();

    AgentId id1 = acs[0]->getLocalAgentId();
    AgentId id2 = acs[1]->getLocalAgentId();
    ASSERT_NE(id1, id2) << "Agents use the same ID.";

    aes[0]->start();
    aes[1]->start();

    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    alicaTests::TestWorldModel::getOne()->robotsXPos.push_back(0);
    alicaTests::TestWorldModel::getOne()->robotsXPos.push_back(2000);

    alicaTests::TestWorldModel::getTwo()->robotsXPos.push_back(2000);
    alicaTests::TestWorldModel::getTwo()->robotsXPos.push_back(0);

    for (int i = 0; i < 21; i++) {
        acs[0]->stepEngine();
        acs[1]->stepEngine();

        if (i == 1) {
            EXPECT_TRUE(alica::test::Util::isStateActive(aes[0], 1414403553717));
            EXPECT_TRUE(alica::test::Util::isStateActive(aes[1], 1414403553717));
        }

        if (i == 20) {
            EXPECT_TRUE(alica::test::Util::isStateActive(aes[0], 1414403553717));
            EXPECT_TRUE(alica::test::Util::isStateActive(aes[1], 1414403429950));
        }
    }

    auto logs = alicaTests::TestWorldModel::getOne()->tracingLogs;

    bool foundTaskAssignmentChangeLog = false;
    for (auto log : logs) {
        if (log.first == "TaskAssignmentChange") {
            foundTaskAssignmentChangeLog = true;
            break;
        }
    }
    EXPECT_TRUE(foundTaskAssignmentChangeLog);
    EXPECT_EQ(alicaTests::TestWorldModel::getOne()->tracingParents["EmptyBehaviour"], "AuthorityTest");
}
} // namespace
} // namespace alica