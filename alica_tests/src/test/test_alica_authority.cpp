#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/IAlicaCommunication.h>
#include <engine/allocationauthority/AllocationDifference.h>
#include <engine/allocationauthority/EntryPointRobotPair.h>
#include <engine/model/Task.h>
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

class AlicaEngineAuthorityManager : public AlicaTestFixtureBase
{
protected:
    alica::AlicaEngine* ae2;

    virtual void SetUp()
    {
        // determine the path to the test config
        ros::NodeHandle nh;
        std::string path;
        nh.param<std::string>("/rootPath", path, ".");
        // bring up the SystemConfig with the corresponding path
        sc = essentials::SystemConfig::getInstance();
        sc->setRootPath(path);
        sc->setConfigPath(path + "/etc");
        sc->setHostname("nase");

        // setup the engine parts
        bc = new alica::BehaviourCreator();
        cc = new alica::ConditionCreator();
        uc = new alica::UtilityFunctionCreator();
        crc = new alica::ConstraintCreator();
    }

    virtual void TearDown()
    {
        ae->shutdown();
        ae2->shutdown();
        sc->shutdown();
        delete ae->getCommunicator();
        delete ae2->getCommunicator();
        delete cc;
        delete bc;
        delete uc;
        delete crc;
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
    alica::Task t1(1, false);
    alica::Task t2(2, false);
    alica::EntryPoint e1(1, nullptr, &t1, nullptr);
    alica::EntryPoint e2(2, nullptr, &t2, nullptr);

    const char* aname = "aa";
    const char* bname = "bb";

    essentials::AgentID a1(reinterpret_cast<const uint8_t*>(aname), 2);
    essentials::AgentID a2(reinterpret_cast<const uint8_t*>(bname), 2);

    alica::EntryPointRobotPair aTot1(&e1, &a1);
    alica::EntryPointRobotPair bTot1(&e1, &a2);
    alica::EntryPointRobotPair aTot2(&e2, &a1);
    alica::EntryPointRobotPair bTot2(&e2, &a2);

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

    sc->setHostname("nase");
    ae = new alica::AlicaEngine(new essentials::AgentIDManager(new essentials::AgentIDFactory()), "RolesetTA", "AuthorityTestMaster", true);
    ae->setAlicaClock(new alica::AlicaClock());
    ae->setCommunicator(new alicaDummyProxy::AlicaDummyCommunication(ae));
    EXPECT_TRUE(ae->init(bc, cc, uc, crc)) << "Unable to initialise the Alica Engine!";

    sc->setHostname("hairy");
    ae2 = new alica::AlicaEngine(new essentials::AgentIDManager(new essentials::AgentIDFactory()), "RolesetTA", "AuthorityTestMaster", true);
    ae2->setAlicaClock(new alica::AlicaClock());
    ae2->setCommunicator(new alicaDummyProxy::AlicaDummyCommunication(ae2));
    EXPECT_TRUE(ae2->init(bc, cc, uc, crc)) << "Unable to initialise the Alica Engine!";

    auto uSummandAe = ae->getPlanRepository()->getPlans().find(1414403413451)->getUtilityFunction()->getUtilSummands()[0].get();
    alica::DummyTestSummand* dbr = dynamic_cast<alica::DummyTestSummand*>(uSummandAe);
    dbr->robotId = ae->getTeamManager()->getLocalAgentID();

    auto uSummandAe2 = ae2->getPlanRepository()->getPlans().find(1414403413451)->getUtilityFunction()->getUtilSummands()[0].get();
    alica::DummyTestSummand* dbr2 = dynamic_cast<alica::DummyTestSummand*>(uSummandAe2);
    dbr2->robotId = ae2->getTeamManager()->getLocalAgentID();

    alica::AgentIDConstPtr id1 = ae->getTeamManager()->getLocalAgentID();
    alica::AgentIDConstPtr id2 = ae2->getTeamManager()->getLocalAgentID();
    ASSERT_NE(id1, id2) << "Agents use the same ID.";

    ae->start();
    ae2->start();

    alicaTests::TestWorldModel::getOne()->robotsXPos.push_back(0);
    alicaTests::TestWorldModel::getOne()->robotsXPos.push_back(2000);

    alicaTests::TestWorldModel::getTwo()->robotsXPos.push_back(2000);
    alicaTests::TestWorldModel::getTwo()->robotsXPos.push_back(0);

    for (int i = 0; i < 21; i++) {
        step(ae);
        step(ae2);

        if (i == 1) {
            EXPECT_EQ(ae->getPlanBase()->getRootNode()->getChildren()[0]->getActiveState()->getId(), 1414403553717);
            EXPECT_EQ(ae2->getPlanBase()->getRootNode()->getChildren()[0]->getActiveState()->getId(), 1414403553717);
        }

        if (i == 20) {
            EXPECT_EQ(ae->getPlanBase()->getRootNode()->getChildren()[0]->getActiveState()->getId(), 1414403553717);
            EXPECT_EQ(ae2->getPlanBase()->getRootNode()->getChildren()[0]->getActiveState()->getId(), 1414403429950);
        }
    }
}
