#include <test_alica.h>
#include <gtest/gtest.h>
#include <engine/AlicaEngine.h>
#include <engine/AlicaClock.h>
#include "engine/IAlicaCommunication.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "UtilityFunctionCreator.h"
#include "BehaviourCreator.h"
#include <communication/AlicaRosCommunication.h>
#include <mutex>
#include <condition_variable>
#include <supplementary/EventTrigger.h>
#include <SystemConfig.h>
#include <engine/BehaviourPool.h>
#include <engine/BasicBehaviour.h>
#include <engine/PlanBase.h>
#include <engine/model/BehaviourConfiguration.h>
#include <TestWorldModel.h>
#include <Plans/Behaviour/TriggerA.h>
#include <Plans/Behaviour/TriggerB.h>
#include <Plans/Behaviour/TriggerC.h>
#include <Plans/Behaviour/NotToTrigger.h>

using namespace std;

class AlicaBehaviourTrigger : public ::testing::Test {
protected:
    supplementary::SystemConfig* sc;
    alica::AlicaEngine* ae;
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
        ae = new alica::AlicaEngine(new supplementary::AgentIDManager(new supplementary::AgentIDFactory()), "Roleset",
                "BehaviourTriggerTestPlan", ".", true);
        bc = new alica::BehaviourCreator();
        cc = new alica::ConditionCreator();
        uc = new alica::UtilityFunctionCreator();
        crc = new alica::ConstraintCreator();
        ae->setAlicaClock(new alica::AlicaClock());
        ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
    }

    virtual void TearDown() {
        ae->shutdown();
        sc->shutdown();
        delete ae->getCommunicator();
        delete cc;
        delete bc;
        delete uc;
        delete crc;
    }
};

TEST_F(AlicaBehaviourTrigger, triggerTest) {
    ASSERT_NE(setjmp(restore_point), SIGSEGV);

    alicaTests::TestWorldModel::getOne()->trigger1 = new supplementary::EventTrigger();
    alicaTests::TestWorldModel::getOne()->trigger2 = new supplementary::EventTrigger();
    ae->init(bc, cc, uc, crc);
    ae->start();
    step(ae);
    for (auto iter : ae->getBehaviourPool()->getAvailableBehaviours()) {
        if (iter.first->getName() == "TriggerA") {
            iter.second->setTrigger(alicaTests::TestWorldModel::getOne()->trigger1);
            continue;
        } else if (iter.first->getName() == "TriggerB") {
            iter.second->setTrigger(alicaTests::TestWorldModel::getOne()->trigger1);
            continue;
        } else if (iter.first->getName() == "TriggerC") {
            iter.second->setTrigger(alicaTests::TestWorldModel::getOne()->trigger2);
            continue;
        } else {
            cout << "BehName: " << iter.first->getName() << endl;
            continue;
        }
    }

    for (auto iter : ae->getBehaviourPool()->getAvailableBehaviours()) {
        if (iter.first->getName() == "TriggerA") {
            EXPECT_EQ(((alica::TriggerA*) (&*iter.second))->callCounter, 0);
            continue;
        } else if (iter.first->getName() == "TriggerB") {
            EXPECT_EQ(((alica::TriggerB*) (&*iter.second))->callCounter, 0);
            continue;
        } else if (iter.first->getName() == "TriggerC") {
            EXPECT_EQ(((alica::TriggerC*) (&*iter.second))->callCounter, 0);
            continue;
        } else if (iter.first->getName() == "NotToTriggerDefault") {
            EXPECT_EQ(((alica::NotToTrigger*) (&*iter.second))->callCounter, 0);
            continue;
        } else {
            cout << iter.first->getName() << endl;
            EXPECT_TRUE(false);
        }
    }
    alicaTests::TestWorldModel::getOne()->trigger1->run();
    alicaTests::TestWorldModel::getOne()->trigger2->run();
    step(ae);
    alicaTests::TestWorldModel::getOne()->trigger1->run();
    alicaTests::TestWorldModel::getOne()->trigger2->run();
    step(ae);
    alicaTests::TestWorldModel::getOne()->trigger1->run();
    alicaTests::TestWorldModel::getOne()->trigger2->run();
    step(ae);
    alicaTests::TestWorldModel::getOne()->trigger2->run();
    step(ae);

    for (auto iter : ae->getBehaviourPool()->getAvailableBehaviours()) {
        if (iter.first->getName() == "TriggerA") {
            EXPECT_EQ(((alica::TriggerA*) (&*iter.second))->callCounter, 3);
            continue;
        } else if (iter.first->getName() == "TriggerB") {
            EXPECT_EQ(((alica::TriggerB*) (&*iter.second))->callCounter, 3);
            continue;
        } else if (iter.first->getName() == "TriggerC") {
            EXPECT_EQ(((alica::TriggerC*) (&*iter.second))->callCounter, 4);
            continue;
        } else if (iter.first->getName() == "NotToTriggerDefault") {
            EXPECT_EQ(((alica::NotToTrigger*) (&*iter.second))->callCounter, 0);
            continue;
        } else {
            EXPECT_TRUE(false);
        }
    }
    cout << "Finished" << endl;
}
