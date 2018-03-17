#include <gtest/gtest.h>

#include <SystemConfig.h>
#include <supplementary/EventTrigger.h>

#include <engine/AlicaEngine.h>
#include <engine/BasicBehaviour.h>
#include <engine/BehaviourPool.h>
#include <engine/IAlicaClock.h>
#include <engine/IAlicaCommunication.h>
#include <engine/PlanBase.h>
#include <engine/model/BehaviourConfiguration.h>

#include <clock/AlicaROSClock.h>
#include <communication/AlicaRosCommunication.h>

#include <BehaviourCreator.h>
#include <ConditionCreator.h>
#include <ConstraintCreator.h>
#include <Plans/Behaviour/NotToTrigger.h>
#include <Plans/Behaviour/TriggerA.h>
#include <Plans/Behaviour/TriggerB.h>
#include <Plans/Behaviour/TriggerC.h>
#include <TestWorldModel.h>
#include <UtilityFunctionCreator.h>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>

class AlicaBehaviourTrigger : public ::testing::Test
{
  protected:
    supplementary::SystemConfig *sc;
    alica::AlicaEngine *ae;
    alica::BehaviourCreator *bc;
    alica::ConditionCreator *cc;
    alica::UtilityFunctionCreator *uc;
    alica::ConstraintCreator *crc;

    virtual void SetUp()
    {
        // determine the path to the test config
        std::string path = supplementary::FileSystem::getSelfPath();
        int place = path.rfind("devel");
        path = path.substr(0, place);
        path = path + "src/alica/alica_tests/src/test";

        // bring up the SystemConfig with the corresponding path
        sc = supplementary::SystemConfig::getInstance();
        sc->setRootPath(path);
        sc->setConfigPath(path + "/etc");
        sc->setHostname("nase");

        // setup the engine
        ae = new alica::AlicaEngine(new supplementary::AgentIDManager(new supplementary::AgentIDFactory()), "Roleset",
                                    "BehaviourTriggerTestPlan", ".", false);
        bc = new alica::BehaviourCreator();
        cc = new alica::ConditionCreator();
        uc = new alica::UtilityFunctionCreator();
        crc = new alica::ConstraintCreator();
        ae->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
        ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
    }

    virtual void TearDown()
    {

        ae->shutdown();
        sc->shutdown();
        delete ae->getIAlicaClock();
        delete ae->getCommunicator();
        delete cc;
        delete bc;
        delete uc;
        delete crc;
    }
};

TEST_F(AlicaBehaviourTrigger, triggerTest)
{
    alicaTests::TestWorldModel::getOne()->trigger1 = new supplementary::EventTrigger();
    alicaTests::TestWorldModel::getOne()->trigger2 = new supplementary::EventTrigger();
    ae->init(bc, cc, uc, crc);
    ae->start();
    std::chrono::milliseconds duration(33);
    this_thread::sleep_for(duration);
    for (auto iter : *ae->getBehaviourPool()->getAvailableBehaviours())
    {
        if (iter.first->getName() == "TriggerA")
        {
            iter.second->setTrigger(alicaTests::TestWorldModel::getOne()->trigger1);
            continue;
        }
        else if (iter.first->getName() == "TriggerB")
        {
            iter.second->setTrigger(alicaTests::TestWorldModel::getOne()->trigger1);
            continue;
        }
        else if (iter.first->getName() == "TriggerC")
        {
            iter.second->setTrigger(alicaTests::TestWorldModel::getOne()->trigger2);
            continue;
        }
        else
        {
            std::cout << "BehName: " << iter.first->getName() << std::endl;
            continue;
        }
    }

    for (auto iter : *ae->getBehaviourPool()->getAvailableBehaviours())
    {
        if (iter.first->getName() == "TriggerA")
        {
            EXPECT_EQ(((alica::TriggerA *)(&*iter.second))->callCounter, 0);
            continue;
        }
        else if (iter.first->getName() == "TriggerB")
        {
            EXPECT_EQ(((alica::TriggerB *)(&*iter.second))->callCounter, 0);
            continue;
        }
        else if (iter.first->getName() == "TriggerC")
        {
            EXPECT_EQ(((alica::TriggerC *)(&*iter.second))->callCounter, 0);
            continue;
        }
        else if (iter.first->getName() == "NotToTriggerDefault")
        {
            EXPECT_EQ(((alica::NotToTrigger *)(&*iter.second))->callCounter, 0);
            continue;
        }
        else
        {
            std::cout << iter.first->getName() << std::endl;
            EXPECT_TRUE(false);
        }
    }
    alicaTests::TestWorldModel::getOne()->trigger1->run();
    alicaTests::TestWorldModel::getOne()->trigger2->run();
    std::this_thread::sleep_for(duration);
    alicaTests::TestWorldModel::getOne()->trigger1->run();
    alicaTests::TestWorldModel::getOne()->trigger2->run();
    std::this_thread::sleep_for(duration);
    alicaTests::TestWorldModel::getOne()->trigger1->run();
    alicaTests::TestWorldModel::getOne()->trigger2->run();
    std::this_thread::sleep_for(duration);
    alicaTests::TestWorldModel::getOne()->trigger2->run();
    std::this_thread::sleep_for(duration);

    for (auto iter : *ae->getBehaviourPool()->getAvailableBehaviours())
    {
        if (iter.first->getName() == "TriggerA")
        {
            EXPECT_EQ(((alica::TriggerA *)(&*iter.second))->callCounter, 3);
            continue;
        }
        else if (iter.first->getName() == "TriggerB")
        {
            EXPECT_EQ(((alica::TriggerB *)(&*iter.second))->callCounter, 3);
            continue;
        }
        else if (iter.first->getName() == "TriggerC")
        {
            EXPECT_EQ(((alica::TriggerC *)(&*iter.second))->callCounter, 4);
            continue;
        }
        else if (iter.first->getName() == "NotToTriggerDefault")
        {
            EXPECT_EQ(((alica::NotToTrigger *)(&*iter.second))->callCounter, 0);
            continue;
        }
        else
        {
            EXPECT_TRUE(false);
        }
    }
    std::cout << "Finished" << std::endl;
}
