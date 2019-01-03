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

class AlicaSyncTransition : public AlicaTestFixtureBase
{ /* namespace alicaTests */
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

        // setup the engine
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
        delete cc;
        delete bc;
        delete uc;
        delete crc;
        delete ae->getCommunicator();
        delete ae2->getCommunicator();
        delete ae;
        delete ae2;
    }
};

/**
 * Test for SyncTransition
 */
TEST_F(AlicaSyncTransition, syncTransitionTest)
{
    ASSERT_NO_SIGNAL

    sc->setHostname("hairy");
    ae = new alica::AlicaEngine(new essentials::AgentIDManager(new essentials::AgentIDFactory()), "RolesetTA", "RealMasterPlanForSyncTest", true);
    ae->setAlicaClock(new alica::AlicaClock());
    ae->setCommunicator(new alicaDummyProxy::AlicaDummyCommunication(ae));
    EXPECT_TRUE(ae->init(bc, cc, uc, crc)) << "Unable to initialise the Alica Engine!";

    sc->setHostname("nase");
    ae2 = new alica::AlicaEngine(new essentials::AgentIDManager(new essentials::AgentIDFactory()), "RolesetTA", "RealMasterPlanForSyncTest", true);
    ae2->setAlicaClock(new alica::AlicaClock());
    ae2->setCommunicator(new alicaDummyProxy::AlicaDummyCommunication(ae2));
    EXPECT_TRUE(ae2->init(bc, cc, uc, crc)) << "Unable to initialise the Alica Engine!";

    ae->start();
    ae2->start();

    for (int i = 0; i < 20; i++) {
        std::cout << i << "AE ----------------------------------------------- " << *ae->getTeamManager()->getLocalAgentID() << std::endl;
        step(ae);

        std::cout << i << "AE ----------------------------------------------- " << *ae2->getTeamManager()->getLocalAgentID() << std::endl;
        step(ae2);

        if (i == 2) {
            alicaTests::TestWorldModel::getOne()->setTransitionCondition1418825427317(true);
            alicaTests::TestWorldModel::getTwo()->setTransitionCondition1418825427317(true);
        }
        if (i == 3) {
            alicaTests::TestWorldModel::getTwo()->setTransitionCondition1418825428924(true);
            alicaTests::TestWorldModel::getOne()->setTransitionCondition1418825428924(true);
        }
        if (i > 1 && i < 4) {
            EXPECT_EQ(ae->getPlanBase()->getRootNode()->getChildren()[0]->getActiveState()->getId(), 1418825395940);
            EXPECT_EQ(ae2->getPlanBase()->getRootNode()->getChildren()[0]->getActiveState()->getId(), 1418825404963);
        }
        if (i == 5) {
            EXPECT_EQ(ae->getPlanBase()->getRootNode()->getChildren()[0]->getActiveState()->getId(), 1418825409988);
            EXPECT_EQ(ae2->getPlanBase()->getRootNode()->getChildren()[0]->getActiveState()->getId(), 1418825411686);
        }
    }
}
