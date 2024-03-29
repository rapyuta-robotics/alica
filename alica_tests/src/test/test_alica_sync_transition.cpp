#include "test_alica.h"

#include <alica_tests/DummyTestSummand.h>
#include <alica_tests/TestWorldModel.h>

#include <engine/IAlicaCommunication.h>
#include <engine/PlanBase.h>
#include <engine/PlanRepository.h>
#include <engine/TeamObserver.h>
#include <engine/UtilityFunction.h>
#include <engine/model/Plan.h>
#include <engine/model/State.h>
#include <engine/teammanager/TeamManager.h>

#include <alica/test/Util.h>
#include <communication/AlicaDummyCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>

#include <gtest/gtest.h>

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
    const char* getHostName(int agentNumber) const override { return agentNumber ? "nase" : "hairy"; }
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
    std::shared_ptr<alicaTests::TestWorldModel> twm1 =
            LockedBlackboardRW(acs[0]->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    std::shared_ptr<alicaTests::TestWorldModel> twm2 =
            LockedBlackboardRW(acs[1]->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");

    for (int i = 0; i < 20; i++) {
        std::cout << i << "AE ----------------------------------------------- " << acs[0]->getLocalAgentId() << std::endl;
        acs[0]->stepEngine();

        std::cout << i << "AE ----------------------------------------------- " << acs[1]->getLocalAgentId() << std::endl;
        acs[1]->stepEngine();

        if (i == 2) {
            twm1->setTransitionCondition1418825427317(true);
            twm2->setTransitionCondition1418825427317(true);
        }
        if (i == 3) {
            twm2->setTransitionCondition1418825428924(true);
            twm1->setTransitionCondition1418825428924(true);
        }
        if (i > 1 && i < 4) {
            EXPECT_TRUE(alica::test::Util::isStateActive(aes[0], 1418825395940));
            EXPECT_TRUE(alica::test::Util::isStateActive(aes[1], 1418825404963));
        }
        if (i == 5) {
            //            std::cout << "TEST Iteration " << i << std::endl;
            EXPECT_TRUE(alica::test::Util::isStateActive(aes[0], 1418825409988));
            EXPECT_TRUE(alica::test::Util::isStateActive(aes[1], 1418825411686));
        }
    }
}
} // namespace
} // namespace alica
