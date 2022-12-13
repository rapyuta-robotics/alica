#include "test_alica.h"

#include <alica_tests/Behaviour/Attack.h>
#include <alica_tests/Behaviour/MidFieldStandard.h>
#include <alica_tests/BehaviourCreator.h>
#include <alica_tests/ConditionCreator.h>
#include <alica_tests/ConstraintCreator.h>
#include <alica_tests/PlanCreator.h>
#include <alica_tests/UtilityFunctionCreator.h>

#include <alica_tests/DummyTestSummand.h>
#include <alica_tests/TestWorldModel.h>

#include <alica/test/Util.h>
#include <communication/AlicaDummyCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/Assignment.h>
#include <engine/BasicBehaviour.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaCommunication.h>
#include <engine/PlanBase.h>
#include <engine/PlanRepository.h>
#include <engine/TeamObserver.h>
#include <engine/UtilityFunction.h>
#include <engine/allocationauthority/AllocationDifference.h>
#include <engine/allocationauthority/EntryPointRobotPair.h>
#include <engine/model/Behaviour.h>
#include <engine/model/Plan.h>
#include <engine/model/RuntimeCondition.h>
#include <engine/model/State.h>
#include <engine/model/Task.h>
#include <engine/modelmanagement/factories/TaskFactory.h>
#include <engine/teammanager/TeamManager.h>

#include <gtest/gtest.h>

namespace alica
{
namespace
{

class AlicaTestFixtureWM : public AlicaTestFixture
{
protected:
    const char* getMasterPlanName() const override { return "ConstraintTestPlan"; }
};

TEST_F(AlicaTestFixtureWM, simpleGetWM)
{
    alicaTests::TestWorldModel* wm1 = ac->getWorldModel<alicaTests::TestWorldModel>("worldModel");
    alicaTests::TestWorldModel* wm2 = ae->getWorldModel<alicaTests::TestWorldModel>("worldModel");
    size_t bbSize = ac->getWorldModels().impl()._worldModels.size();

    EXPECT_NE(nullptr, wm1);
    EXPECT_NE(nullptr, wm2);
    EXPECT_EQ(1, bbSize);
}

TEST_F(AlicaTestFixtureWM, multipleGetWM)
{
    ac->addWorldModelByType<alicaTests::TestWorldModel>("worldModel1");
    ac->addWorldModelByType<alicaTests::TestWorldModel>("worldModel2");

    alicaTests::TestWorldModel* wm1 = ac->getWorldModel<alicaTests::TestWorldModel>("worldModel");
    alicaTests::TestWorldModel* wm2 = ae->getWorldModel<alicaTests::TestWorldModel>("worldModel");
    size_t bbSize = ac->getWorldModels().impl()._worldModels.size();

    EXPECT_NE(nullptr, wm1);
    EXPECT_NE(nullptr, wm2);
    EXPECT_EQ(3, bbSize);

    wm1 = ac->getWorldModel<alicaTests::TestWorldModel>("worldModel1");
    wm2 = ae->getWorldModel<alicaTests::TestWorldModel>("worldModel1");

    EXPECT_NE(nullptr, wm1);
    EXPECT_NE(nullptr, wm2);

    wm1 = ac->getWorldModel<alicaTests::TestWorldModel>("worldModel2");
    wm2 = ae->getWorldModel<alicaTests::TestWorldModel>("worldModel2");

    EXPECT_NE(nullptr, wm1);
    EXPECT_NE(nullptr, wm2);
}

} // namespace
} // namespace alica
