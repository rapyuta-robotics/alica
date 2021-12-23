#include "test_alica.h"

#include "engine/SimplePlanTree.h"
#include "engine/containers/PlanTreeInfo.h"
#include "alica_tests/TestWorldModel.h"
#include <alica/test/Util.h>
#include <engine/RunningPlan.h>

#include <gtest/gtest.h>

namespace alica
{

class AlicaSerializationTest : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "SerializationMasterPlan"; }
    bool stepEngine() const override { return false; }

    static constexpr long planAPlanId = 1433931143598606082;
    static constexpr long planBPlanId = 230205985761632608;
    static constexpr long planCPlanId = 2359124678252958039;
    static constexpr long planDPlanId = 1781630225028158279;

    static constexpr long emptyPlanStateId = 3164334534532883889;
    static constexpr long planAStateId = 1059656669948994292;
    static constexpr long planBStateId = 1137921287745324120;
    static constexpr long planCStateId = 2604890859274745239;
    static constexpr long planDStateId = 4372755713553523771;

    static constexpr long serializationTestAStateId = 4556827380180239242;
    static constexpr long serializationTestBStateId = 174185769149002104;
    static constexpr long serializationTestCStateId = 458399185905834888;
    static constexpr long serializationTestDStateId = 837657643540052235;

    void waitUntilPlan(long planId)
    {
        alica::AlicaTime sleepTime = alica::AlicaTime::seconds(1);
        do {
            ae->getAlicaClock().sleep(sleepTime);
        } while (!alica::test::Util::isPlanActive(ae, planId));
    }

    // Calls private methods inside TeamObserver
    std::unique_ptr<SimplePlanTree> serializeAndDeserialize(IdGrp msg_to_send)
    {
        // Serialize (see TeamObserver::doBroadCast)
        PlanTreeInfo pti = ae->getTeamObserver().sptToMessage(msg_to_send);

        // De-serialize (see TeamObserver::updateTeamPlanTrees and TeamObserver::handlePlanTreeInfo)
        std::unique_ptr<SimplePlanTree> spi = ae->getTeamObserver().sptFromMessage(pti.senderID, pti.dynamicStateIDPairs, ae->getAlicaClock().now());
        return spi;
    }
};

/**
 * Tests whether plan serialization works fine
 */
TEST_F(AlicaSerializationTest, serializeDeserializeA)
{
    ASSERT_NO_SIGNAL
    ae->start();
    auto* worldModel = dynamic_cast<alicaTests::TestWorldModel*>(ae->getWorldModel());
    worldModel->serializationTestA = true;

    waitUntilPlan(planAPlanId);
    const RunningPlan* rpRoot = ae->getPlanBase().getRootNode();
    ASSERT_TRUE(rpRoot);

    // Build current Plan Tree
    IdGrp msg_to_send;
    int deepestNode = 0;
    int treeDepth = 0;
    rpRoot->toMessage(msg_to_send, rpRoot, deepestNode, treeDepth);
    ASSERT_EQ(msg_to_send.size(), 15);
    ASSERT_EQ(msg_to_send.at(0), 0);
    ASSERT_EQ(msg_to_send.at(1), serializationTestAStateId);
    ASSERT_EQ(msg_to_send.at(2), 0);
    ASSERT_EQ(msg_to_send.at(3), planAStateId);
    ASSERT_EQ(msg_to_send.at(4), 0);
    ASSERT_EQ(msg_to_send.at(5), emptyPlanStateId);
    ASSERT_EQ(msg_to_send.at(6), -1);
    ASSERT_EQ(msg_to_send.at(7), -1);
    ASSERT_EQ(msg_to_send.at(8), 0);
    ASSERT_EQ(msg_to_send.at(9), emptyPlanStateId);
    ASSERT_EQ(msg_to_send.at(10), -1);
    ASSERT_EQ(msg_to_send.at(11), 0);
    ASSERT_EQ(msg_to_send.at(12), emptyPlanStateId);
    ASSERT_EQ(msg_to_send.at(13), -1);
    ASSERT_EQ(msg_to_send.at(14), -1);
    ASSERT_EQ(msg_to_send.size(), 15);

    std::unique_ptr<SimplePlanTree> spi = serializeAndDeserialize(msg_to_send);
    ASSERT_TRUE(spi);

    IdGrp msg_received = spi->getDynamicStateIDPairs();
    ASSERT_EQ(msg_to_send, msg_received);
}

/**
 * Tests whether plan serialization works fine
 */
TEST_F(AlicaSerializationTest, serializeDeserializeB)
{
    ASSERT_NO_SIGNAL
    ae->start();
    auto* worldModel = dynamic_cast<alicaTests::TestWorldModel*>(ae->getWorldModel());
    worldModel->serializationTestB = true;

    waitUntilPlan(planBPlanId);
    const RunningPlan* rpRoot = ae->getPlanBase().getRootNode();
    ASSERT_TRUE(rpRoot);

    // Build current Plan Tree
    IdGrp msg_to_send;
    int deepestNode = 0;
    int treeDepth = 0;
    rpRoot->toMessage(msg_to_send, rpRoot, deepestNode, treeDepth);
    ASSERT_EQ(msg_to_send.size(), 15);
    ASSERT_EQ(msg_to_send.at(0), 0);
    ASSERT_EQ(msg_to_send.at(1), serializationTestBStateId);
    ASSERT_EQ(msg_to_send.at(2), 0);
    ASSERT_EQ(msg_to_send.at(3), planBStateId);
    ASSERT_EQ(msg_to_send.at(4), 0);
    ASSERT_EQ(msg_to_send.at(5), emptyPlanStateId);
    ASSERT_EQ(msg_to_send.at(6), -1);
    ASSERT_EQ(msg_to_send.at(7), 0);
    ASSERT_EQ(msg_to_send.at(8), emptyPlanStateId);
    ASSERT_EQ(msg_to_send.at(9), -1);
    ASSERT_EQ(msg_to_send.at(10), 0);
    ASSERT_EQ(msg_to_send.at(11), emptyPlanStateId);
    ASSERT_EQ(msg_to_send.at(12), -1);
    ASSERT_EQ(msg_to_send.at(13), -1);
    ASSERT_EQ(msg_to_send.at(14), -1);

    std::unique_ptr<SimplePlanTree> spi = serializeAndDeserialize(msg_to_send);
    ASSERT_TRUE(spi);

    IdGrp msg_received = spi->getDynamicStateIDPairs();
    ASSERT_EQ(msg_to_send, msg_received);
}

/**
 * Tests whether plan serialization works fine
 */
TEST_F(AlicaSerializationTest, serializeDeserializeC)
{
    ASSERT_NO_SIGNAL
    ae->start();
    auto* worldModel = dynamic_cast<alicaTests::TestWorldModel*>(ae->getWorldModel());
    worldModel->serializationTestC = true;

    waitUntilPlan(planCPlanId);
    const RunningPlan* rpRoot = ae->getPlanBase().getRootNode();
    ASSERT_TRUE(rpRoot);

    // Build current Plan Tree
    IdGrp msg_to_send;
    int deepestNode = 0;
    int treeDepth = 0;
    rpRoot->toMessage(msg_to_send, rpRoot, deepestNode, treeDepth);

    ASSERT_EQ(msg_to_send.size(), 27);
    ASSERT_EQ(msg_to_send.at(0), 0);
    ASSERT_EQ(msg_to_send.at(1), serializationTestCStateId);
    ASSERT_EQ(msg_to_send.at(2), 0);
    ASSERT_EQ(msg_to_send.at(3), planCStateId);
    ASSERT_EQ(msg_to_send.at(4), 0);
    ASSERT_EQ(msg_to_send.at(5), planBStateId);
    ASSERT_EQ(msg_to_send.at(6), 0);
    ASSERT_EQ(msg_to_send.at(7), emptyPlanStateId);
    ASSERT_EQ(msg_to_send.at(8), -1);
    ASSERT_EQ(msg_to_send.at(9), 0);
    ASSERT_EQ(msg_to_send.at(10), emptyPlanStateId);
    ASSERT_EQ(msg_to_send.at(11), -1);
    ASSERT_EQ(msg_to_send.at(12), 0);
    ASSERT_EQ(msg_to_send.at(13), emptyPlanStateId);
    ASSERT_EQ(msg_to_send.at(14), -1);
    ASSERT_EQ(msg_to_send.at(15), -1);
    ASSERT_EQ(msg_to_send.at(16), 0);
    ASSERT_EQ(msg_to_send.at(17), planAStateId);
    ASSERT_EQ(msg_to_send.at(18), 0);
    ASSERT_EQ(msg_to_send.at(19), emptyPlanStateId);
    ASSERT_EQ(msg_to_send.at(20), -1);
    ASSERT_EQ(msg_to_send.at(21), -1);
    ASSERT_EQ(msg_to_send.at(22), 0);
    ASSERT_EQ(msg_to_send.at(23), emptyPlanStateId);
    ASSERT_EQ(msg_to_send.at(24), -1);
    ASSERT_EQ(msg_to_send.at(25), -1);
    ASSERT_EQ(msg_to_send.at(26), -1);

    std::unique_ptr<SimplePlanTree> spi = serializeAndDeserialize(msg_to_send);
    ASSERT_TRUE(spi);

    IdGrp msg_received = spi->getDynamicStateIDPairs();
    ASSERT_EQ(msg_to_send, msg_received);
}

/**
 * Tests whether plan serialization works fine
 */
TEST_F(AlicaSerializationTest, serializeDeserializeD)
{
    ASSERT_NO_SIGNAL
    ae->start();
    auto* worldModel = dynamic_cast<alicaTests::TestWorldModel*>(ae->getWorldModel());
    worldModel->serializationTestD = true;

    waitUntilPlan(planDPlanId);
    const RunningPlan* rpRoot = ae->getPlanBase().getRootNode();
    ASSERT_TRUE(rpRoot);

    // Build current Plan Tree
    IdGrp msg_to_send;
    int deepestNode = 0;
    int treeDepth = 0;
    rpRoot->toMessage(msg_to_send, rpRoot, deepestNode, treeDepth);

    ASSERT_EQ(msg_to_send.size(), 12);
    ASSERT_EQ(msg_to_send.at(0), 0);
    ASSERT_EQ(msg_to_send.at(1), serializationTestDStateId);
    ASSERT_EQ(msg_to_send.at(2), 0);
    ASSERT_EQ(msg_to_send.at(3), emptyPlanStateId);
    ASSERT_EQ(msg_to_send.at(4), -1);
    ASSERT_NE(msg_to_send.at(5), 0); // dynamic EP id is not 0
    ASSERT_EQ(msg_to_send.at(6), planDStateId);
    ASSERT_EQ(msg_to_send.at(7), 0);
    ASSERT_EQ(msg_to_send.at(8), emptyPlanStateId);
    ASSERT_EQ(msg_to_send.at(9), -1);
    ASSERT_EQ(msg_to_send.at(10), -1);
    ASSERT_EQ(msg_to_send.at(11), -1);

    std::unique_ptr<SimplePlanTree> spi = serializeAndDeserialize(msg_to_send);
    ASSERT_TRUE(spi);

    IdGrp msg_received = spi->getDynamicStateIDPairs();
    ASSERT_EQ(msg_to_send, msg_received);
}


} // namespace alica
