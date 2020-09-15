#include "test_alica.h"

#include <Behaviour/NotToTrigger.h>
#include <Behaviour/TriggerA.h>
#include <Behaviour/TriggerB.h>
#include <Behaviour/TriggerC.h>
#include <TestWorldModel.h>
#include <alica/test/Util.h>
#include <engine/BasicBehaviour.h>
#include <engine/BehaviourPool.h>
#include <engine/IAlicaCommunication.h>
#include <engine/PlanBase.h>
#include <engine/model/ConfAbstractPlanWrapper.h>
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
    EXPECT_EQ(std::dynamic_pointer_cast<alica::TriggerA>(alica::test::Util::getBasicBehaviour(ae, 1428508297492, 0))->callCounter, 0);
    EXPECT_EQ(std::dynamic_pointer_cast<alica::TriggerB>(alica::test::Util::getBasicBehaviour(ae, 1428508316905, 0))->callCounter, 0);
    EXPECT_EQ(std::dynamic_pointer_cast<alica::TriggerC>(alica::test::Util::getBasicBehaviour(ae, 1428508355209, 0))->callCounter, 0);
    EXPECT_EQ(std::dynamic_pointer_cast<alica::NotToTrigger>(alica::test::Util::getBasicBehaviour(ae, 1429017274116, 0))->callCounter, 0);

    ae->start();
    alica::AlicaTime duration = alica::AlicaTime::milliseconds(100);
    ae->getAlicaClock().sleep(duration);

    EXPECT_EQ(std::dynamic_pointer_cast<alica::TriggerA>(alica::test::Util::getBasicBehaviour(ae, 1428508297492, 0))->callCounter, 0);
    EXPECT_EQ(std::dynamic_pointer_cast<alica::TriggerB>(alica::test::Util::getBasicBehaviour(ae, 1428508316905, 0))->callCounter, 0);
    EXPECT_EQ(std::dynamic_pointer_cast<alica::TriggerC>(alica::test::Util::getBasicBehaviour(ae, 1428508355209, 0))->callCounter, 0);
    EXPECT_EQ(std::dynamic_pointer_cast<alica::NotToTrigger>(alica::test::Util::getBasicBehaviour(ae, 1429017274116, 0))->callCounter, 0);

    alicaTests::TestWorldModel::getOne()->trigger1->run(true);
    alicaTests::TestWorldModel::getOne()->trigger2->run(true);

    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(33));

    alicaTests::TestWorldModel::getOne()->trigger1->run(true);
    alicaTests::TestWorldModel::getOne()->trigger2->run(true);

    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(33));

    alicaTests::TestWorldModel::getOne()->trigger1->run(true);
    alicaTests::TestWorldModel::getOne()->trigger2->run(true);

    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(33));

    alicaTests::TestWorldModel::getOne()->trigger2->run(true);

    ae->getAlicaClock().sleep(duration * 2);

    EXPECT_EQ(std::dynamic_pointer_cast<alica::TriggerA>(alica::test::Util::getBasicBehaviour(ae, 1428508297492, 0))->callCounter, 3);
    EXPECT_EQ(std::dynamic_pointer_cast<alica::TriggerB>(alica::test::Util::getBasicBehaviour(ae, 1428508316905, 0))->callCounter, 3);
    EXPECT_EQ(std::dynamic_pointer_cast<alica::TriggerC>(alica::test::Util::getBasicBehaviour(ae, 1428508355209, 0))->callCounter, 4);
    EXPECT_EQ(std::dynamic_pointer_cast<alica::NotToTrigger>(alica::test::Util::getBasicBehaviour(ae, 1429017274116, 0))->callCounter, 0);
}
} // namespace
} // namespace alica