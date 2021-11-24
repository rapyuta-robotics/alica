
#include "engine/DefaultUtilityFunction.h"
#include "engine/IAlicaCommunication.h"
#include "engine/PlanRepository.h"
#include "engine/model/Behaviour.h"
#include "engine/model/Plan.h"
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <alica_tests/TestWorldModel.h>
#include <gtest/gtest.h>
#include <test_alica.h>

namespace alica
{
namespace
{

class AlicaEngineTestPlanPool : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "MasterPlan"; }
};

class AlicaEngineTestPlanPoolConfigs : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "PlanPoolTestMasterPlan"; }
};

/**
 * Tests the initialisation of the planPool
 */
TEST_F(AlicaEngineTestPlanPool, planPoolInit)
{
    ASSERT_NO_SIGNAL

    for (const alica::Plan* plan : ae->getPlanRepository().getPlans()) {
        ASSERT_NE(plan, nullptr);
        std::cout << "Plan: " << plan->getName() << std::endl;
    }
}

TEST_F(AlicaEngineTestPlanPoolConfigs, planPoolCheckPlanConfigs)
{
    ASSERT_NO_SIGNAL
    auto* wm = dynamic_cast<alicaTests::TestWorldModel*>(ac->getWorldModel());
    wm->configParameter.clear();
    wm->setTransitionCondition4238964946542987247(false);

    ae->start();
    ac->stepEngine();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));

    wm->setTransitionCondition4238964946542987247(true);
    ac->stepEngine();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));

    ASSERT_EQ(wm->configParameter[0], "1");
    ASSERT_EQ(wm->configParameter[1], "2");
}

}
}