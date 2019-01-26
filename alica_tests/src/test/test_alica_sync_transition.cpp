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
    virtual const char* getRoleSetName() const override { return "RolesetTA"; }
    virtual const char* getMasterPlanName() const override { return "RealMasterPlanForSyncTest"; }
    virtual int getAgentCount() const override { return agentCount; }
    virtual const char* getHostName(int agentNumber) const override
    {
        if (agentNumber) {
            return "nase";
        } else {
            return "hairy";
        }
    }

    virtual void SetUp() override { AlicaTestMultiAgentFixture::SetUp(); }

    virtual void TearDown() override { AlicaTestMultiAgentFixture::TearDown(); }
};

/**
 * Test for SyncTransition
 */
TEST_F(AlicaSyncTransition, syncTransitionTest)
{
    ASSERT_NO_SIGNAL

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