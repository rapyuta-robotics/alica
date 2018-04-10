#include <gtest/gtest.h>
#include <engine/AlicaEngine.h>
#include <engine/AlicaClock.h>
#include "engine/IAlicaCommunication.h"
#include "engine/model/State.h"
#include "engine/model/Behaviour.h"
#include "engine/PlanRepository.h"
#include "engine/BasicBehaviour.h"
#include "engine/BehaviourPool.h"
#include "engine/PlanBase.h"
#include <engine/AlicaClock.h>
#include <communication/AlicaRosCommunication.h>
#include "engine/DefaultUtilityFunction.h"
#include "engine/TeamObserver.h"
#include "engine/model/Plan.h"
#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "UtilityFunctionCreator.h"
#include "engine/Assignment.h"
#include "engine/collections/AssignmentCollection.h"
#include "engine/collections/StateCollection.h"
#include "TestWorldModel.h"
#include <supplementary/AgentIDManager.h>
#include <Plans/Behaviour/Attack.h>

class AlicaMultiAgent : public ::testing::Test {
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
        std::cout << "rootPath " << path << std::endl;
        // bring up the SystemConfig with the corresponding path
        sc = supplementary::SystemConfig::getInstance();
        sc->setRootPath(path);
        sc->setConfigPath(path + "/etc");
        // setup the engine
        bc = new alica::BehaviourCreator();
        cc = new alica::ConditionCreator();
        uc = new alica::UtilityFunctionCreator();
        crc = new alica::ConstraintCreator();
    }

    virtual void TearDown() {
        ae->shutdown();
        ae2->shutdown();
        delete ae->getCommunicator();
        delete ae2->getCommunicator();
        sc->shutdown();
        delete cc;
        delete bc;
        delete uc;
        delete crc;
    }
};
/**
 * Tests whether it is possible to use multiple agents.
 */
TEST_F(AlicaMultiAgent, runMultiAgentPlan) {
    sc->setHostname("nase");
    ae = new alica::AlicaEngine(new supplementary::AgentIDManager(new supplementary::AgentIDFactory()), "RolesetTA",
            "MultiAgentTestMaster", ".", true);
    ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
    ASSERT_TRUE(ae->init(bc, cc, uc, crc)) << "Unable to initialise the Alica Engine!";

    sc->setHostname("hairy");
    ae2 = new alica::AlicaEngine(new supplementary::AgentIDManager(new supplementary::AgentIDFactory()), "RolesetTA",
            "MultiAgentTestMaster", ".", true);
    ae2->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae2));
    ASSERT_TRUE(ae2->init(bc, cc, uc, crc)) << "Unable to initialise the Alica Engine!";

    ae->start();
    ae2->start();
    chrono::milliseconds duration(33);
    while (!ae->getPlanBase()->isWaiting() || !ae2->getPlanBase()->isWaiting()) {
        this_thread::sleep_for(duration);
    }

    for (int i = 0; i < 20; i++) {
        ASSERT_TRUE(ae->getPlanBase()->isWaiting());
        ASSERT_TRUE(ae2->getPlanBase()->isWaiting());
        ae->stepNotify();

        this_thread::sleep_for(duration);

        ae2->stepNotify();
        this_thread::sleep_for(duration);
        while (!ae->getPlanBase()->isWaiting() || !ae2->getPlanBase()->isWaiting()) {
            this_thread::sleep_for(duration);
        }
        //        if (i > 24)
        //        {
        //            if (ae->getPlanBase()->getDeepestNode() != nullptr)
        //                cout << "AE: " << ae->getPlanBase()->getDeepestNode()->toString() << endl;
        //            if (ae2->getPlanBase()->getDeepestNode() != nullptr)
        //                cout << "AE2: " << ae2->getPlanBase()->getDeepestNode()->toString() << endl;
        //            cout << "-------------------------" << endl;
        //        }

        if (i < 10) {
            ASSERT_EQ(ae->getPlanBase()->getRootNode()->getActiveState()->getId(), 1413200842974);
            ASSERT_EQ(ae2->getPlanBase()->getRootNode()->getActiveState()->getId(), 1413200842974);
        }
        if (i == 10) {
            cout << "1--------- Initial State passed ---------" << endl;
            alicaTests::TestWorldModel::getOne()->setTransitionCondition1413201227586(true);
            alicaTests::TestWorldModel::getTwo()->setTransitionCondition1413201227586(true);
        }
        if (i > 11 && i < 15) {
            ASSERT_EQ(ae->getPlanBase()->getRootNode()->getActiveState()->getId(), 1413201213955);
            ASSERT_EQ(ae2->getPlanBase()->getRootNode()->getActiveState()->getId(), 1413201213955);
            ASSERT_EQ((*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getPlan()->getName(),
                    string("MultiAgentTestPlan"));
            ASSERT_EQ((*ae2->getPlanBase()->getRootNode()->getChildren()->begin())->getPlan()->getName(),
                    string("MultiAgentTestPlan"));
        }
        if (i == 15) {
            for (auto iter : *ae->getBehaviourPool()->getAvailableBehaviours()) {
                if (iter.second->getName() == "Attack") {
                    ASSERT_GT(((alica::Attack*) &*iter.second)->callCounter, 5);
                    if (((alica::Attack*) &*iter.second)->callCounter > 3) {
                        alicaTests::TestWorldModel::getOne()->setTransitionCondition1413201052549(true);
                        alicaTests::TestWorldModel::getTwo()->setTransitionCondition1413201052549(true);
                        alicaTests::TestWorldModel::getOne()->setTransitionCondition1413201370590(true);
                        alicaTests::TestWorldModel::getTwo()->setTransitionCondition1413201370590(true);
                    }
                }
            }
            cout << "2--------- Engagement to cooperative plan passed ---------" << endl;
        }
        if (i == 16) {
            ASSERT_TRUE((*ae2->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId() ==
                                1413201030936 ||
                        (*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId() ==
                                1413201030936)
                    << endl
                    << (*ae2->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId() << " "
                    << (*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId() << endl;

            ASSERT_TRUE((*ae2->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId() ==
                                1413807264574 ||
                        (*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId() ==
                                1413807264574)
                    << endl
                    << (*ae2->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId() << " "
                    << (*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId() << endl;
            alicaTests::TestWorldModel::getOne()->setTransitionCondition1413201227586(false);
            alicaTests::TestWorldModel::getTwo()->setTransitionCondition1413201227586(false);
            cout << "3--------- Passed transitions in subplan passed ---------" << endl;
        }
        if (i >= 17 && i <= 18) {
            ASSERT_TRUE((*ae2->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId() ==
                                1413201030936 ||
                        (*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId() ==
                                1413201030936)
                    << "AE State: "
                    << (*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId()
                    << " AE2 State: "
                    << (*ae2->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId() << endl;
            ASSERT_TRUE((*ae2->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId() ==
                                1413807264574 ||
                        (*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId() ==
                                1413807264574)
                    << "AE State: "
                    << (*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId() << " "
                    << (*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->toString() << endl
                    << " AE2 State: "
                    << (*ae2->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->getId() << " "
                    << (*ae2->getPlanBase()->getRootNode()->getChildren()->begin())->getActiveState()->toString()
                    << endl;
            if (i == 18) {
                cout << "4--------- Stayed in these state although previous transitions are not true anymore ---------"
                     << endl;
                alicaTests::TestWorldModel::getOne()->setTransitionCondition1413201389955(true);
                alicaTests::TestWorldModel::getTwo()->setTransitionCondition1413201389955(true);
            }
        }
        if (i == 19) {
            ASSERT_TRUE(ae2->getPlanBase()->getRootNode()->getActiveState()->getId() == 1413201380359 &&
                        ae->getPlanBase()->getRootNode()->getActiveState()->getId() == 1413201380359)
                    << " AE State: " << ae->getPlanBase()->getRootNode()->getActiveState()->getId()
                    << " AE2 State: " << ae2->getPlanBase()->getRootNode()->getActiveState()->getId() << endl;
        }
    }
}
