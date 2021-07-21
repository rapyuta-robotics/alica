#include "test_alica.h"

#include "Behaviour/Attack.h"
#include "Behaviour/MidFieldStandard.h"
#include "CounterClass.h"
#include "DummyTestSummand.h"
#include "TestWorldModel.h"

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

#include <gtest/gtest.h>

namespace alica
{
namespace
{

class EngineRulesTestPlan : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "RolesetTA"; }
    const char* getMasterPlanName() const override { return "EngineRulesSchedulingTestMaster"; }
    bool stepEngine() const override { return false; }
};

class AlicaEngineAuthorityManager : public AlicaTestMultiAgentFixture
{
protected:
    const int agentCount = 2;
    const char* getRoleSetName() const override { return "RolesetTA"; }
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

TEST_F(EngineRulesTestPlan, dynamicAllocationTest)
{
    ASSERT_NO_SIGNAL
    ae->start();
    alica::scheduler::Scheduler& scheduler = ae->editScheduler();
    scheduler.terminate();

    alica::AlicaTime sleepTime = alica::AlicaTime::seconds(1);
    uint8_t timeoutCount = 0;
    alicaTests::TestWorldModel::getOne()->setTransitionCondition1625783867495(true);
    EXPECT_TRUE(alicaTests::TestWorldModel::getOne()->isTransitionCondition1625783867495());

    ae->getAlicaClock().sleep(sleepTime);

    EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1625614677498)) << "Agent is not in first state of master plan!";
    EXPECT_TRUE(alica::test::Util::isPlanActive(ae, 1625614640417)) << "Agent is not in EngineRulesSchedulingTestPlan plan!";
    EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1625614714499)) << "Agent is not in first state of EngineRulesSchedulingTestPlan!";

    alicaTests::TestWorldModel::getOne()->setTransitionCondition1625614729978(true);
    alicaTests::TestWorldModel::getOne()->setSwitchingEntryPoints(true);

    ae->getAlicaClock().sleep(sleepTime);
    alicaTests::TestWorldModel::getOne()->setSwitchingEntryPoints(false);

    bool isPrerequisite = false;
    for (std::weak_ptr<alica::scheduler::Job> weakJob : scheduler.getPrerequisites(13)) {
        std::shared_ptr<alica::scheduler::Job> sharedJob = weakJob.lock();
        if (sharedJob && sharedJob->id == 12) {
            isPrerequisite = true;
            break;
        }
    }
    ASSERT_TRUE(isPrerequisite);

    isPrerequisite = false;
    for (std::weak_ptr<alica::scheduler::Job> weakJob : scheduler.getPrerequisites(12)) {
        std::shared_ptr<alica::scheduler::Job> sharedJob = weakJob.lock();
        if (sharedJob && sharedJob->id == 11) {
            isPrerequisite = true;
            break;
        }
    }
    ASSERT_TRUE(isPrerequisite);

    alicaTests::TestWorldModel::getOne()->setTransitionCondition1626848015861(true);
    ae->getAlicaClock().sleep(sleepTime);
}

TEST_F(EngineRulesTestPlan, redoTest)
{
    ASSERT_NO_SIGNAL
    alicaTests::TestWorldModel::getOne()->setTransitionCondition1626848015861(false);
    alicaTests::TestWorldModel::getOne()->setSwitchingEntryPoints(false);
    alicaTests::TestWorldModel::getOne()->setTransitionCondition1625783869825(false);
    ae->start();
    alica::scheduler::Scheduler& scheduler = ae->editScheduler();
    scheduler.terminate();

    alica::AlicaTime sleepTime = alica::AlicaTime::seconds(1);
    uint8_t timeoutCount = 0;
    alicaTests::TestWorldModel::getOne()->setTransitionCondition1625783867495(true);
    EXPECT_TRUE(alicaTests::TestWorldModel::getOne()->isTransitionCondition1625783867495());

    ae->getAlicaClock().sleep(sleepTime);

    EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1625614677498)) << "Agent is not in first state of master plan!";
    EXPECT_TRUE(alica::test::Util::isPlanActive(ae, 1625614640417)) << "Agent is not in EngineRulesSchedulingTestPlan plan!";
    EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1625614714499)) << "Agent is not in first state of EngineRulesSchedulingTestPlan!";

    alicaTests::TestWorldModel::getOne()->setTransitionCondition1625776897472(true);
    ae->getAlicaClock().sleep(sleepTime);

    EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1625614677498)) << "Agent is not in first state of EngineRulesSchedulingTestPlan!";

    ae->getAlicaClock().sleep(sleepTime);

    bool isPrerequisite = false;
    for (std::weak_ptr<alica::scheduler::Job> weakJob : scheduler.getPrerequisites(6)) {
        std::shared_ptr<alica::scheduler::Job> sharedJob = weakJob.lock();
        if (sharedJob && sharedJob->id == 5) {
            isPrerequisite = true;
            break;
        }
    }
    ASSERT_TRUE(isPrerequisite);
}

TEST_F(EngineRulesTestPlan, planReplaceTest)
{
    ASSERT_NO_SIGNAL
    alicaTests::TestWorldModel::getOne()->setTransitionCondition1626848015861(false);
    alicaTests::TestWorldModel::getOne()->setSwitchingEntryPoints(false);
    alicaTests::TestWorldModel::getOne()->setTransitionCondition1625783869825(false);
    ae->start();
    alica::scheduler::Scheduler& scheduler = ae->editScheduler();
    scheduler.terminate();

    alica::AlicaTime sleepTime = alica::AlicaTime::seconds(1);
    uint8_t timeoutCount = 0;
    alicaTests::TestWorldModel::getOne()->setTransitionCondition1625783867495(true);
    EXPECT_TRUE(alicaTests::TestWorldModel::getOne()->isTransitionCondition1625783867495());

    ae->getAlicaClock().sleep(sleepTime);

    EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1625614677498)) << "Agent is not in first state of master plan!";
    EXPECT_TRUE(alica::test::Util::isPlanActive(ae, 1625614640417)) << "Agent is not in EngineRulesSchedulingTestPlan plan!";
    EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1625614714499)) << "Agent is not in first state of EngineRulesSchedulingTestPlan!";

    alicaTests::TestWorldModel::getOne()->setTransitionCondition1625776897472(true);
    ae->getAlicaClock().sleep(sleepTime);

    EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1625614677498)) << "Agent is not in first state of EngineRulesSchedulingTestPlan!";

    alicaTests::TestWorldModel::getOne()->setTransitionCondition1625776897472(true);
    ae->getAlicaClock().sleep(sleepTime);

    EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1625614677498)) << "Agent is not in first state of EngineRulesSchedulingTestPlan!";

    // init of new plan depends on terminate of old plan
    bool isPrerequisite = false;
    for (std::weak_ptr<alica::scheduler::Job> weakJob : scheduler.getPrerequisites(10)) {
        std::shared_ptr<alica::scheduler::Job> sharedJob = weakJob.lock();
        if (sharedJob && sharedJob->id == 9) {
            isPrerequisite = true;
            break;
        }
    }
    ASSERT_TRUE(isPrerequisite);

    // terminate of old plan depends on terminate of behaviour
    isPrerequisite = false;
    for (std::weak_ptr<alica::scheduler::Job> weakJob : scheduler.getPrerequisites(9)) {
        std::shared_ptr<alica::scheduler::Job> sharedJob = weakJob.lock();
        if (sharedJob && sharedJob->id == 8) {
            isPrerequisite = true;
            break;
        }
    }
    ASSERT_TRUE(isPrerequisite);
}

TEST_F(EngineRulesTestPlan, topFailTest)
{
    ASSERT_NO_SIGNAL
    alicaTests::TestWorldModel::getOne()->setTransitionCondition1626848015861(false);
    alicaTests::TestWorldModel::getOne()->setTransitionCondition1625783867495(false);
    ae->start();
    alica::scheduler::Scheduler& scheduler = ae->editScheduler();
    scheduler.terminate();

    alica::AlicaTime sleepTime = alica::AlicaTime::seconds(1);
    uint8_t timeoutCount = 0;
    alicaTests::TestWorldModel::getOne()->setTransitionCondition1625783869825(true);

    ae->getAlicaClock().sleep(sleepTime);

    bool isPrerequisite = false;
    for (std::weak_ptr<alica::scheduler::Job> weakJob : scheduler.getPrerequisites(5)) {
        std::shared_ptr<alica::scheduler::Job> sharedJob = weakJob.lock();
        if (sharedJob && sharedJob->id == 4) {
            isPrerequisite = true;
            break;
        }
    }
    ASSERT_TRUE(isPrerequisite);
}

TEST_F(AlicaEngineAuthorityManager, authorityOverrideRule)
{
    // ASSERT_NO_SIGNAL
    alicaTests::TestWorldModel::getOne()->setTransitionCondition1626848015861(false);

    const Plan* plan = aes[0]->getPlanRepository().getPlans().find(1414403413451);
    ASSERT_NE(plan, nullptr) << "Plan 1414403413451 is unknown";
    ASSERT_NE(plan->getUtilityFunction(), nullptr) << "UtilityFunction is null!";

    auto uSummandAe = plan->getUtilityFunction()->getUtilSummands()[0].get();
    alica::DummyTestSummand* dbr = dynamic_cast<alica::DummyTestSummand*>(uSummandAe);
    dbr->robotId = acs[0]->getLocalAgentId();

    auto uSummandAe2 = aes[1]->getPlanRepository().getPlans().find(1414403413451)->getUtilityFunction()->getUtilSummands()[0].get();
    alica::DummyTestSummand* dbr2 = dynamic_cast<alica::DummyTestSummand*>(uSummandAe2);
    dbr2->robotId = acs[1]->getLocalAgentId();

    essentials::IdentifierConstPtr id1 = acs[0]->getLocalAgentId();
    essentials::IdentifierConstPtr id2 = acs[1]->getLocalAgentId();
    ASSERT_NE(id1, id2) << "Agents use the same ID.";

    aes[0]->start();
    aes[1]->start();

    alica::scheduler::Scheduler& scheduler1 = aes[0]->editScheduler();
    scheduler1.terminate();
    alica::scheduler::Scheduler& scheduler2 = aes[1]->editScheduler();
    scheduler2.terminate();

    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());

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

    bool isPrerequisite = false;
    for (std::weak_ptr<alica::scheduler::Job> weakJob : scheduler2.getPrerequisites(6)) {
        std::shared_ptr<alica::scheduler::Job> sharedJob = weakJob.lock();
        if (sharedJob && sharedJob->id == 5) {
            isPrerequisite = true;
            break;
        }
    }
    ASSERT_TRUE(isPrerequisite);

    isPrerequisite = false;
    for (std::weak_ptr<alica::scheduler::Job> weakJob : scheduler2.getPrerequisites(7)) {
        std::shared_ptr<alica::scheduler::Job> sharedJob = weakJob.lock();
        if (sharedJob && sharedJob->id == 6) {
            isPrerequisite = true;
            break;
        }
    }
    ASSERT_TRUE(isPrerequisite);
}

} // namespace
} // namespace alica