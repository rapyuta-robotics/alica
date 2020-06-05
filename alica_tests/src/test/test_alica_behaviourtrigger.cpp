#include "test_alica.h"

#include "engine/IAlicaCommunication.h"
#include "engine/model/ConfAbstractPlanWrapper.h"
#include <Behaviour/NotToTrigger.h>
#include <Behaviour/TriggerA.h>
#include <Behaviour/TriggerB.h>
#include <Behaviour/TriggerC.h>
#include <TestWorldModel.h>
#include <engine/BasicBehaviour.h>
#include <engine/BehaviourPool.h>
#include <engine/PlanBase.h>
#include <essentials/EventTrigger.h>

#include <condition_variable>
#include <mutex>

namespace alica
{
namespace
{

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
    ae->getAlicaClock().sleep(duration);

    for (auto iter : ae->getBehaviourPool().getAvailableBehaviours()) {
        if (iter.first->getAbstractPlan()->getName() == "TriggerA") {
            iter.second->setTrigger(alicaTests::TestWorldModel::getOne()->trigger1);
            continue;
        } else if (iter.first->getAbstractPlan()->getName() == "TriggerB") {
            iter.second->setTrigger(alicaTests::TestWorldModel::getOne()->trigger1);
            continue;
        } else if (iter.first->getAbstractPlan()->getName() == "TriggerC") {
            iter.second->setTrigger(alicaTests::TestWorldModel::getOne()->trigger2);
            continue;
        } else {
            std::cout << "BehName: " << iter.first->getName() << std::endl;
            continue;
        }
    }

    for (auto iter : ae->getBehaviourPool().getAvailableBehaviours()) {
        if (iter.first->getAbstractPlan()->getName() == "TriggerA") {
            EXPECT_EQ(((alica::TriggerA*) (&*iter.second))->callCounter, 0);
            continue;
        } else if (iter.first->getAbstractPlan()->getName() == "TriggerB") {
            EXPECT_EQ(((alica::TriggerB*) (&*iter.second))->callCounter, 0);
            continue;
        } else if (iter.first->getAbstractPlan()->getName() == "TriggerC") {
            EXPECT_EQ(((alica::TriggerC*) (&*iter.second))->callCounter, 0);
            continue;
        } else if (iter.first->getAbstractPlan()->getName() == "NotToTrigger") {
            EXPECT_EQ(((alica::NotToTrigger*) (&*iter.second))->callCounter, 0);
            continue;
        } else {
            std::cout << iter.first->getAbstractPlan()->getName() << std::endl;
            EXPECT_TRUE(false);
        }
    }
    alicaTests::TestWorldModel::getOne()->trigger1->run();
    alicaTests::TestWorldModel::getOne()->trigger2->run();

    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(33));

    alicaTests::TestWorldModel::getOne()->trigger1->run();
    alicaTests::TestWorldModel::getOne()->trigger2->run();

    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(33));

    alicaTests::TestWorldModel::getOne()->trigger1->run();
    alicaTests::TestWorldModel::getOne()->trigger2->run();

    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(33));

    alicaTests::TestWorldModel::getOne()->trigger2->run();

    ae->getAlicaClock().sleep(duration * 2);

    for (auto iter : ae->getBehaviourPool().getAvailableBehaviours()) {
        if (iter.first->getAbstractPlan()->getName() == "TriggerA") {
            EXPECT_EQ(((alica::TriggerA*) (&*iter.second))->callCounter, 3);
            continue;
        } else if (iter.first->getAbstractPlan()->getName() == "TriggerB") {
            EXPECT_EQ(((alica::TriggerB*) (&*iter.second))->callCounter, 3);
            continue;
        } else if (iter.first->getAbstractPlan()->getName() == "TriggerC") {
            EXPECT_EQ(((alica::TriggerC*) (&*iter.second))->callCounter, 4);
            continue;
        } else if (iter.first->getAbstractPlan()->getName() == "NotToTrigger") {
            EXPECT_EQ(((alica::NotToTrigger*) (&*iter.second))->callCounter, 0);
            continue;
        } else {
            EXPECT_TRUE(false);
        }
    }
    std::cout << "Finished" << std::endl;
}
}
}