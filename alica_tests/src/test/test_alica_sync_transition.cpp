#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "DummyTestSummand.h"
#include "TestConstantValueSummand.h"
#include "TestWorldModel.h"
#include "UtilityFunctionCreator.h"
#include "engine/IAlicaCommunication.h"
#include "engine/PlanBase.h"
#include "engine/PlanRepository.h"
#include "engine/TeamObserver.h"
#include "engine/UtilityFunction.h"
#include "engine/model/Plan.h"
#include "engine/model/State.h"
#include "engine/teammanager/TeamManager.h"
#include <communication/AlicaDummyCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
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
    tcs[0]->startEngine();
    tcs[1]->startEngine();
    // Allow agents to discover each other
    tcs[0]->getAlicaClock().sleep(getDiscoveryTimeout());

    for (int i = 0; i < 20; i++) {
        std::cout << i << "AE ----------------------------------------------- " << *(tcs[0]->getLocalAgentId()) << std::endl;
        tcs[0]->stepEngine();

        std::cout << i << "AE ----------------------------------------------- " << *(tcs[1]->getLocalAgentId()) << std::endl;
        tcs[1]->stepEngine();

        if (i == 2) {
            alicaTests::TestWorldModel::getOne()->setTransitionCondition1418825427317(true);
            alicaTests::TestWorldModel::getTwo()->setTransitionCondition1418825427317(true);
        }
        if (i == 3) {
            alicaTests::TestWorldModel::getTwo()->setTransitionCondition1418825428924(true);
            alicaTests::TestWorldModel::getOne()->setTransitionCondition1418825428924(true);
        }
        if (i > 1 && i < 4) {
            EXPECT_TRUE(tcs[0]->isStateActive(1418825395940));
            EXPECT_TRUE(tcs[1]->isStateActive(1418825404963));
        }
        if (i == 5) {
//            std::cout << "TEST Iteration " << i << std::endl;
            EXPECT_TRUE(tcs[0]->isStateActive(1418825409988));
            EXPECT_TRUE(tcs[1]->isStateActive(1418825411686));
        }
    }
}
}
}