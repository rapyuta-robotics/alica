#include "test_alica.h"

#include <alica_tests/BehaviourCreator.h>
#include <alica_tests/ConditionCreator.h>
#include <alica_tests/ConstraintCreator.h>
#include <alica_tests/DummyTestSummand.h>
#include <alica_tests/PlanCreator.h>
#include <alica_tests/TestWorldModel.h>
#include <alica_tests/UtilityFunctionCreator.h>
#include <alica_tests/TransitionConditionCreator.h>

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
#include <engine/IAlicaCommunication.h>
#include <engine/allocationauthority/AllocationDifference.h>
#include <engine/allocationauthority/EntryPointRobotPair.h>
#include <engine/model/Task.h>
#include <engine/modelmanagement/factories/TaskFactory.h>

#include <gtest/gtest.h>

namespace alica
{
namespace
{

class AlicaEngineAuthorityManager : public AlicaTestMultiAgentFixture
{
protected:
    AlicaEngineAuthorityManager(){};
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

    AgentId a1 = 1;
    AgentId a2 = 2;

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
    const Plan* plan = aes[0]->getPlanRepository().getPlans().find(1414403413451);
    ASSERT_NE(plan, nullptr) << "Plan 1414403413451 is unknown";
    ASSERT_NE(plan->getUtilityFunction(), nullptr) << "UtilityFunction is null!";
    auto uSummandAe = plan->getUtilityFunction()->getUtilSummands()[0].get();
    alica::DummyTestSummand* dbr = dynamic_cast<alica::DummyTestSummand*>(uSummandAe);
    dbr->robotId = acs[0]->getLocalAgentId();

    auto uSummandAe2 = aes[1]->getPlanRepository().getPlans().find(1414403413451)->getUtilityFunction()->getUtilSummands()[0].get();
    alica::DummyTestSummand* dbr2 = dynamic_cast<alica::DummyTestSummand*>(uSummandAe2);
    dbr2->robotId = acs[1]->getLocalAgentId();

    AgentId id1 = acs[0]->getLocalAgentId();
    AgentId id2 = acs[1]->getLocalAgentId();
    ASSERT_NE(id1, id2) << "Agents use the same ID.";

    aes[0]->start();
    aes[1]->start();

    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());

    auto* wmOne = dynamic_cast<alicaTests::TestWorldModel*>(acs[0]->getWorldModel());
    auto* wmTwo = dynamic_cast<alicaTests::TestWorldModel*>(acs[1]->getWorldModel());

    wmOne->robotsXPos.push_back(0);
    wmOne->robotsXPos.push_back(2000);

    wmTwo->robotsXPos.push_back(2000);
    wmTwo->robotsXPos.push_back(0);

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
}
} // namespace
} // namespace alica
