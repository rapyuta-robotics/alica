#include <gtest/gtest.h>
#include <engine/AlicaEngine.h>
#include <engine/AlicaClock.h>
#include "engine/IAlicaCommunication.h"
#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "UtilityFunctionCreator.h"
#include <communication/AlicaRosCommunication.h>
#include "TestWorldModel.h"
#include "engine/PlanRepository.h"
#include "engine/UtilityFunction.h"
#include "engine/model/Plan.h"
#include "TestConstantValueSummand.h"
#include "engine/TeamObserver.h"
#include "engine/PlanBase.h"
#include "engine/model/State.h"
#include "TestWorldModel.h"
#include "DummyTestSummand.h"
#include "engine/teammanager/TeamManager.h"

class AlicaSyncTransition : public ::testing::Test { /* namespace alicaTests */
protected:
    supplementary::SystemConfig* sc;
    alica::AlicaEngine* ae;
    alica::AlicaEngine* ae2;
    alica::BehaviourCreator* bc;
    alica::ConditionCreator* cc;
    alica::UtilityFunctionCreator* uc;
    alica::ConstraintCreator* crc;
    alicaRosProxy::AlicaRosCommunication* ros;
    alicaRosProxy::AlicaRosCommunication* ros2;
    
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
        bc = new alica::BehaviourCreator();
        cc = new alica::ConditionCreator();
        uc = new alica::UtilityFunctionCreator();
        crc = new alica::ConstraintCreator();
    }

    virtual void TearDown() {
        ae->shutdown();
        ae2->shutdown();
        sc->shutdown();
        delete cc;
        delete bc;
        delete uc;
        delete crc;
        delete ae;
        delete ae2;
        delete ros;
        delete ros2;
    }

    static void step(alica::AlicaEngine* ae) {
        ae->stepNotify();
        do {
            ae->getAlicaClock()->sleep(AlicaTime::milliseconds(33));
        } while (!ae->getPlanBase()->isWaiting());
    }
};

/**
 * Test for SyncTransition
 */
TEST_F(AlicaSyncTransition, syncTransitionTest) {
    sc->setHostname("hairy");
    ae = new alica::AlicaEngine(new supplementary::AgentIDManager(new supplementary::AgentIDFactory()), "RolesetTA",
            "RealMasterPlanForSyncTest", ".", true);
    ros = new alicaRosProxy::AlicaRosCommunication(ae);
    ae->setAlicaClock(new alica::AlicaClock());
    ae->setCommunicator(ros);
    EXPECT_TRUE(ae->init(bc, cc, uc, crc)) << "Unable to initialise the Alica Engine!";

    sc->setHostname("nase");
    ae2 = new alica::AlicaEngine(new supplementary::AgentIDManager(new supplementary::AgentIDFactory()), "RolesetTA",
            "RealMasterPlanForSyncTest", ".", true);
    ros2 = new alicaRosProxy::AlicaRosCommunication(ae2);
    ae2->setAlicaClock(new alica::AlicaClock());
    ae2->setCommunicator(ros2);
    EXPECT_TRUE(ae2->init(bc, cc, uc, crc)) << "Unable to initialise the Alica Engine!";

    ae->start();
    ae2->start();

    for (int i = 0; i < 20; i++) {
        std::cout << "AE ----------------------------------------------- " << *ae->getTeamManager()->getLocalAgentID()
                  << std::endl;
        step(ae);

        std::cout << "AE ----------------------------------------------- " << *ae2->getTeamManager()->getLocalAgentID()
                  << std::endl;
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
            EXPECT_EQ((*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId(),
                    1418825395940);
            EXPECT_EQ((*ae2->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId(),
                    1418825404963);
        }
        if (i == 5) {
            EXPECT_EQ((*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId(),
                    1418825409988);
            EXPECT_EQ((*ae2->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId(),
                    1418825411686);
        }
    }
}
