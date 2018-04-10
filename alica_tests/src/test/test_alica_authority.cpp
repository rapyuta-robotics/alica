#include <gtest/gtest.h>
#include <engine/AlicaEngine.h>
#include <engine/AlicaClock.h>
#include <engine/IAlicaCommunication.h>
#include <engine/allocationauthority/AllocationDifference.h>
#include <engine/allocationauthority/EntryPointRobotPair.h>
#include <engine/model/Task.h>

#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "UtilityFunctionCreator.h"
#include <engine/AlicaClock.h>
#include <communication/AlicaRosCommunication.h>
#include "TestWorldModel.h"
#include "engine/PlanRepository.h"
#include "engine/UtilityFunction.h"
#include "engine/model/Plan.h"
#include "DummyTestSummand.h"
#include "engine/TeamObserver.h"
#include "engine/teammanager/TeamManager.h"
#include "engine/PlanBase.h"
#include "engine/model/State.h"

class AlicaEngineAuthorityManager : public ::testing::Test {
protected:
    supplementary::SystemConfig* sc;
    alica::AlicaEngine* ae;
    alica::AlicaEngine* ae2;
    alica::BehaviourCreator* bc;
    alica::ConditionCreator* cc;
    alica::UtilityFunctionCreator* uc;
    alica::ConstraintCreator* crc;

    virtual void SetUp() {
        // determine the path to the test config
        ros::NodeHandle nh;
        std::string path;
        nh.param<std::string>("/rootPath", path, ".");
        // bring up the SystemConfig with the corresponding path
        sc = supplementary::SystemConfig::getInstance();
        sc->setRootPath(path);
        sc->setConfigPath(path + "/etc");
        sc->setHostname("nase");

        // setup the engine
        ae = new alica::AlicaEngine(new supplementary::AgentIDManager(new supplementary::AgentIDFactory()), "RolesetTA",
                "AuthorityTestMaster", ".", true);
        bc = new alica::BehaviourCreator();
        cc = new alica::ConditionCreator();
        uc = new alica::UtilityFunctionCreator();
        crc = new alica::ConstraintCreator();
        ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
    }

    virtual void TearDown() {
        ae->shutdown();
        sc->shutdown();
        ae2->shutdown();
        delete ae->getCommunicator();
        delete ae2->getCommunicator();
        delete cc;
        delete bc;
        delete uc;
        delete crc;
    }
};

TEST(AllocationDifference, MessageCancelsUtil) {
    alica::AllocationDifference util;
    alica::AllocationDifference msg;
    alica::AllocationDifference result;
    util.setReason(AllocationDifference::Reason::utility);
    msg.setReason(AllocationDifference::Reason::message);
    Task t1(1,false);
    Task t2(2,false);
    EntryPoint e1(1, nullptr, &t1, nullptr);
    EntryPoint e2(2, nullptr, &t2, nullptr);

    const char* aname = "aa";
    const char* bname = "bb";

    supplementary::AgentID a1(reinterpret_cast<const uint8_t*>(aname), 2);
    supplementary::AgentID a2(reinterpret_cast<const uint8_t*>(bname), 2);

    EntryPointRobotPair aTot1(&e1, &a1);
    EntryPointRobotPair bTot1(&e1, &a2);
    EntryPointRobotPair aTot2(&e2, &a1);
    EntryPointRobotPair bTot2(&e2, &a2);

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

TEST_F(AlicaEngineAuthorityManager, authority) {
    sc->setHostname("nase");
    ae = new alica::AlicaEngine(new supplementary::AgentIDManager(new supplementary::AgentIDFactory()), "RolesetTA",
            "AuthorityTestMaster", ".", true);
    ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
    EXPECT_TRUE(ae->init(bc, cc, uc, crc)) << "Unable to initialise the Alica Engine!";

    sc->setHostname("hairy");
    ae2 = new alica::AlicaEngine(new supplementary::AgentIDManager(new supplementary::AgentIDFactory()), "RolesetTA",
            "AuthorityTestMaster", ".", true);
    ae2->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae2));
    EXPECT_TRUE(ae2->init(bc, cc, uc, crc)) << "Unable to initialise the Alica Engine!";

    auto uSummandAe = *(ae->getPlanRepository()->getPlans().find(1414403413451)
                                ->getUtilityFunction()
                                ->getUtilSummands()
                                .begin());
    DummyTestSummand* dbr = dynamic_cast<DummyTestSummand*>(uSummandAe);
    dbr->robotId = ae->getTeamManager()->getLocalAgentID();

    auto uSummandAe2 = *(ae2->getPlanRepository()->getPlans().find(1414403413451)
                                 ->getUtilityFunction()
                                 ->getUtilSummands()
                                 .begin());
    DummyTestSummand* dbr2 = dynamic_cast<DummyTestSummand*>(uSummandAe2);
    dbr2->robotId = ae2->getTeamManager()->getLocalAgentID();

    const supplementary::AgentID* id1 = ae->getTeamManager()->getLocalAgentID();
    const supplementary::AgentID* id2 = ae2->getTeamManager()->getLocalAgentID();
    ASSERT_NE(*id1, *id2) << "Agents use the same ID.";

    ae->start();
    ae2->start();

    alicaTests::TestWorldModel::getOne()->robotsXPos.push_back(0);
    alicaTests::TestWorldModel::getOne()->robotsXPos.push_back(2000);

    alicaTests::TestWorldModel::getTwo()->robotsXPos.push_back(2000);
    alicaTests::TestWorldModel::getTwo()->robotsXPos.push_back(0);

    for (int i = 0; i < 21; i++) {
        ae->stepNotify();
        chrono::milliseconds duration(33);
        this_thread::sleep_for(duration);
        ae2->stepNotify();
        this_thread::sleep_for(duration);
        while (!ae->getPlanBase()->isWaiting() || !ae2->getPlanBase()->isWaiting()) {
            this_thread::sleep_for(duration);
        }

        if (i == 1) {
            EXPECT_EQ((*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId(),
                    1414403553717);
            EXPECT_EQ((*ae2->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId(),
                    1414403553717);
        }

        if (i == 20) {
            EXPECT_EQ((*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId(),
                    1414403553717);
            EXPECT_EQ((*ae2->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId(),
                    1414403429950);
        }
    }
}
