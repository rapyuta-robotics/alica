#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/IAlicaCommunication.h>
#include <engine/allocationauthority/AllocationDifference.h>
#include <engine/allocationauthority/EntryPointRobotPair.h>
#include <engine/model/Task.h>
#include <engine/modelmanagement/factories/TaskFactory.h>
#include <gtest/gtest.h>
#include <test_alica.h>

#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "DummyTestSummand.h"
#include "TestWorldModel.h"
#include "UtilityFunctionCreator.h"
#include "engine/PlanBase.h"
#include "engine/PlanRepository.h"
#include "engine/TeamObserver.h"
#include "engine/UtilityFunction.h"
#include "engine/model/Plan.h"
#include "engine/model/State.h"
#include "engine/teammanager/TeamManager.h"
#include <communication/AlicaDummyCommunication.h>

namespace alica
{
namespace
{

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

TEST(AllocationDifference, MessageCancelsUtil)
{
    ASSERT_NO_SIGNAL

    alica::AllocationDifference util;
    alica::AllocationDifference msg;
    alica::AllocationDifference result;
    util.setReason(alica::AllocationDifference::Reason::utility);
    msg.setReason(alica::AllocationDifference::Reason::message);
    alica::Task t1;
    alica::Task t2;
    alica::EntryPoint e1(1, nullptr, &t1, nullptr);
    alica::EntryPoint e2(2, nullptr, &t2, nullptr);

    essentials::IDManager idManager;
    int idA1 = 1;
    int idA2 = 2;
    essentials::IdentifierConstPtr a1 = idManager.getID<int>(idA1);
    essentials::IdentifierConstPtr a2 = idManager.getID<int>(idA2);

    alica::EntryPointRobotPair aTot1(&e1, a1);
    alica::EntryPointRobotPair bTot1(&e1, a2);
    alica::EntryPointRobotPair aTot2(&e2, a1);
    alica::EntryPointRobotPair bTot2(&e2, a2);

    ASSERT_EQ(a1, a1);
    ASSERT_NE(a1, a2);

    ASSERT_EQ(aTot1, aTot1);
    ASSERT_NE(aTot1, aTot2);
    ASSERT_NE(aTot1, bTot1);

    util.editAdditions().push_back(aTot1);
    util.editSubtractions().push_back(aTot2);

    msg.editAdditions().push_back(aTot2);
    msg.editSubtractions().push_back(aTot1);

    result.applyDifference(util);
    EXPECT_FALSE(result.isEmpty());
    result.applyDifference(msg);
    EXPECT_TRUE(result.isEmpty());
}

TEST_F(AlicaEngineAuthorityManager, authority)
{
    // ASSERT_NO_SIGNAL
    const Plan* plan = tcs[0]->getPlan(1414403413451);
    ASSERT_NE(plan, nullptr) << "Plan 1414403413451 is unknown";
    ASSERT_NE(plan->getUtilityFunction(), nullptr) << "UtilityFunction is null!";
    auto uSummandAe = plan->getUtilityFunction()->getUtilSummands()[0].get();
    alica::DummyTestSummand* dbr = dynamic_cast<alica::DummyTestSummand*>(uSummandAe);
    dbr->robotId = tcs[0]->getLocalAgentId();

    auto uSummandAe2 = tcs[1]->getPlan(1414403413451)->getUtilityFunction()->getUtilSummands()[0].get();
    alica::DummyTestSummand* dbr2 = dynamic_cast<alica::DummyTestSummand*>(uSummandAe2);
    dbr2->robotId = tcs[1]->getLocalAgentId();

    essentials::IdentifierConstPtr id1 = tcs[0]->getLocalAgentId();
    essentials::IdentifierConstPtr id2 = tcs[1]->getLocalAgentId();
    ASSERT_NE(id1, id2) << "Agents use the same ID.";

    tcs[0]->startEngine();
    tcs[1]->startEngine();

    tcs[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    alicaTests::TestWorldModel::getOne()->robotsXPos.push_back(0);
    alicaTests::TestWorldModel::getOne()->robotsXPos.push_back(2000);

    alicaTests::TestWorldModel::getTwo()->robotsXPos.push_back(2000);
    alicaTests::TestWorldModel::getTwo()->robotsXPos.push_back(0);

    for (int i = 0; i < 21; i++) {
        tcs[0]->stepEngine();
        tcs[1]->stepEngine();

        if (i == 1) {
            EXPECT_TRUE(tcs[0]->isStateActive(1414403553717));
            EXPECT_TRUE(tcs[1]->isStateActive(1414403553717));
        }

        if (i == 20) {
            EXPECT_TRUE(tcs[0]->isStateActive(1414403553717));
            EXPECT_TRUE(tcs[1]->isStateActive(1414403429950));
        }
    }
}
}
}