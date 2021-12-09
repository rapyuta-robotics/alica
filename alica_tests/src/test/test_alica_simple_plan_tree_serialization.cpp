#include "test_alica.h"

#include "engine/SimplePlanTree.h"
#include "engine/containers/PlanTreeInfo.h"
#include <alica/test/Util.h>
#include <engine/RunningPlan.h>

#include <gtest/gtest.h>

namespace alica
{

class AlicaSerializationTest : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "SimpleTestPlan"; }
    bool stepEngine() const override { return false; }

    static constexpr long SimpleTestPlanId = 1412252439925;
    static constexpr long TestState2Id = 1412761855746;

    void waitUntilPlan(long planId)
    {
        alica::AlicaTime sleepTime = alica::AlicaTime::seconds(1);
        do {
            ae->getAlicaClock().sleep(sleepTime);
        } while (!alica::test::Util::isPlanActive(ae, SimpleTestPlanId));
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
TEST_F(AlicaSerializationTest, serializeDeserialize)
{
    ASSERT_NO_SIGNAL
    ae->start();
    waitUntilPlan(SimpleTestPlanId);
    const RunningPlan* rpRoot = ae->getPlanBase().getRootNode();
    ASSERT_TRUE(rpRoot);

    // Build current Plan Tree
    IdGrp msg_to_send;
    int deepestNode = 0;
    int treeDepth = 0;
    rpRoot->toMessage(msg_to_send, rpRoot, deepestNode, treeDepth);
    ASSERT_EQ(msg_to_send.size(), 3);
    ASSERT_EQ(msg_to_send.at(0), 0);
    ASSERT_EQ(msg_to_send.at(1), TestState2Id);
    ASSERT_EQ(msg_to_send.at(2), -1);

    std::unique_ptr<SimplePlanTree> spi = serializeAndDeserialize(msg_to_send);
    ASSERT_TRUE(spi);

    IdGrp msg_received = spi->getDynamicStateIDPairs();
    ASSERT_EQ(msg_to_send, msg_received);
}

} // namespace alica
