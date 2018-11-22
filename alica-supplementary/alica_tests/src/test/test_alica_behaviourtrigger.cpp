#include "test_alica.h"

#include "engine/IAlicaCommunication.h"
#include <Plans/Behaviour/NotToTrigger.h>
#include <Plans/Behaviour/TriggerA.h>
#include <Plans/Behaviour/TriggerB.h>
#include <Plans/Behaviour/TriggerC.h>
#include <TestWorldModel.h>
#include <engine/BasicBehaviour.h>
#include <engine/BehaviourPool.h>
#include <engine/PlanBase.h>
#include <engine/model/BehaviourConfiguration.h>
#include <supplementary/EventTrigger.h>

#include <condition_variable>
#include <mutex>

class AlicaBehaviourTrigger : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "BehaviourTriggerTestPlan"; }
    bool stepEngine() const override { return false; }
};

TEST_F(AlicaBehaviourTrigger, triggerTest)
{
    ASSERT_NO_SIGNAL

    ae->start();

    alica::AlicaTime duration = alica::AlicaTime::milliseconds(100);
    ae->getAlicaClock()->sleep(duration);

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

    ae->getAlicaClock()->sleep(alica::AlicaTime::milliseconds(33));

    alicaTests::TestWorldModel::getOne()->trigger1->run();
    alicaTests::TestWorldModel::getOne()->trigger2->run();

    ae->getAlicaClock()->sleep(alica::AlicaTime::milliseconds(33));

    alicaTests::TestWorldModel::getOne()->trigger1->run();
    alicaTests::TestWorldModel::getOne()->trigger2->run();

    ae->getAlicaClock()->sleep(alica::AlicaTime::milliseconds(33));

    alicaTests::TestWorldModel::getOne()->trigger2->run();

    ae->getAlicaClock()->sleep(duration * 2);

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
    std::cout << "Finished" << std::endl;
}
