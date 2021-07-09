#include "test_alica.h"

#include "Behaviour/Attack.h"
#include "Behaviour/MidFieldStandard.h"
#include "CounterClass.h"
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
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "EngineRulesSchedulingTestMaster"; }
    bool stepEngine() const override { return false; }
};

TEST_F(EngineRulesTestPlan, dynamicAllocationTest)
{
    ASSERT_NO_SIGNAL
    ae->start();
    alica::scheduler::Scheduler& scheduler = ae->editScheduler();
    scheduler.terminate();

    alica::AlicaTime sleepTime = alica::AlicaTime::seconds(1);
    uint8_t timeoutCount = 0;

    ae->getAlicaClock().sleep(sleepTime);

    EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1625614677498)) << "Agent is not in first state of master plan!";
    EXPECT_TRUE(alica::test::Util::isPlanActive(ae, 1625614640417)) << "Agent is not in EngineRulesSchedulingTestPlan plan!";
    EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1625614714499)) << "Agent is not in first state of EngineRulesSchedulingTestPlan!";

    alicaTests::TestWorldModel::getOne()->setTransitionCondition1625614729978(true);

    ae->getAlicaClock().sleep(sleepTime);

    EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1625614719367)) << "Agent is not in second state of EngineRulesSchedulingTestPlan!";
}

TEST_F(EngineRulesTestPlan, redoTest)
{
    ASSERT_NO_SIGNAL
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

TEST_F(EngineRulesTestPlan, topFailTest)
{
    ASSERT_NO_SIGNAL
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
        std::cerr << "id: " << sharedJob->id << std::endl;
    }
    ASSERT_TRUE(isPrerequisite);
}

} // namespace
} // namespace alica