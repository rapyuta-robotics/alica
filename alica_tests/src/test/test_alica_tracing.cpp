#include "test_alica.h"

#include "Behaviour/Attack.h"
#include "Behaviour/MidFieldStandard.h"
#include <alica_tests/CounterClass.h>

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

class AlicaTracingTest : public AlicaTestTracingFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "TestTracingMasterPlan"; }
    bool stepEngine() const override { return false; }
};

TEST_F(AlicaTracingTest, runTracing)
{
    ASSERT_NO_SIGNAL
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(200));

    alicaTests::TestWorldModel::getOne()->setPreCondition1840401110297459509(true);
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(200));

    auto logs = alicaTests::TestWorldModel::getOne()->tracingLogs;

    size_t expected_num_of_logs = 9;
    EXPECT_EQ(logs.size(), expected_num_of_logs);

    auto log = logs.front(); // MasterPlan
    EXPECT_EQ(log.first, "Plan");
    EXPECT_EQ(log.second, "true");
    logs.pop();

    log = logs.front(); // MasterPlan Init
    EXPECT_EQ(log.first, "Init");
    EXPECT_EQ(log.second, "true");
    logs.pop();

    log = logs.front(); // SubPlan
    EXPECT_EQ(log.first, "Plan");
    EXPECT_EQ(log.second, "true");
    logs.pop();

    log = logs.front(); // SubPlan Init
    EXPECT_EQ(log.first, "Init");
    EXPECT_EQ(log.second, "true");
    logs.pop();

    log = logs.front(); // Behaviour
    EXPECT_EQ(log.first, "Behaviour");
    EXPECT_EQ(log.second, "true");
    logs.pop();

    log = logs.front(); // Behaviour Init
    EXPECT_EQ(log.first, "Init");
    EXPECT_EQ(log.second, "true");
    logs.pop();

    log = logs.front(); // Behaviour Run
    EXPECT_EQ(log.first, "Run");
    EXPECT_EQ(log.second, "true");
    logs.pop();

    log = logs.front(); // Behaviour Terminate
    EXPECT_EQ(log.first, "Terminate");
    EXPECT_EQ(log.second, "true");
    logs.pop();

    log = logs.front(); // SubPlan Terminate
    EXPECT_EQ(log.first, "Terminate");
    EXPECT_EQ(log.second, "true");
    logs.pop();
}
} // namespace
} // namespace alica