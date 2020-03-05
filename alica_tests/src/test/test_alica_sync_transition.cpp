#include <TestConstantValueSummand.h>
#include <TestWorldModel.h>
#include <engine/IAlicaCommunication.h>
#include <engine/UtilityFunction.h>
#include <engine/model/State.h>
#include <engine/AlicaClock.h>
#include <gtest/gtest.h>
#include <test_alica.h>

namespace alica
{
namespace
{

class AlicaSyncTransition : public AlicaTestMultiAgentFixture
{
protected:
    const int agentCount = 2;
    const char* getRoleSetName() const override { return "RolesetTA"; }
    const char* getMasterPlanName() const override { return "RealMasterPlanForSyncTest"; }
    int getAgentCount() const override { return agentCount; }
    const char* getHostName(int agentNumber) const override
    {
        if (agentNumber) {
            return "nase";
        } else {
            return "hairy";
        }
    }
};

/**
 * Test for SyncTransition
 */
TEST_F(AlicaSyncTransition, syncTransitionTest)
{
    ASSERT_NO_SIGNAL

    aes[0]->start();
    aes[1]->start();
    // Allow agents to discover each other
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());

    for (int i = 0; i < 20; i++) {
        std::cout << i << "AE ----------------------------------------------- " << *(aes[0]->getTeamManager().getLocalAgentID()) << std::endl;
        step(aes[0]);

        std::cout << i << "AE ----------------------------------------------- " << *(aes[1]->getTeamManager().getLocalAgentID()) << std::endl;
        step(aes[1]);

        if (i == 2) {
            alicaTests::TestWorldModel::getOne()->setTransitionCondition1418825427317(true);
            alicaTests::TestWorldModel::getTwo()->setTransitionCondition1418825427317(true);
        }
        if (i == 3) {
            alicaTests::TestWorldModel::getTwo()->setTransitionCondition1418825428924(true);
            alicaTests::TestWorldModel::getOne()->setTransitionCondition1418825428924(true);
        }
        if (i > 1 && i < 4) {
            EXPECT_EQ(aes[0]->getPlanBase().getRootNode()->getChildren()[0]->getActiveState()->getId(), 1418825395940);
            EXPECT_EQ(aes[1]->getPlanBase().getRootNode()->getChildren()[0]->getActiveState()->getId(), 1418825404963);
        }
        if (i == 5) {
            EXPECT_EQ(aes[0]->getPlanBase().getRootNode()->getChildren()[0]->getActiveState()->getId(), 1418825409988);
            EXPECT_EQ(aes[1]->getPlanBase().getRootNode()->getChildren()[0]->getActiveState()->getId(), 1418825411686);
        }
    }
}
}
}