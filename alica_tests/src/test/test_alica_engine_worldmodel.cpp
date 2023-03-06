#include "test_alica.h"

#include <alica_tests/behaviours/Attack.h>
#include <alica_tests/behaviours/MidFieldStandard.h>

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
    std::shared_ptr<alicaTests::TestWorldModel> wm1 =
            LockedBlackboardRW(ac->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    EXPECT_NE(nullptr, wm1);

    LockedBlackboardRW(ac->editGlobalBlackboard()).set("worldmodel2", std::make_shared<alicaTests::TestWorldModel>());
    std::shared_ptr<alicaTests::TestWorldModel> wm2 =
            LockedBlackboardRW(ac->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel2");
    EXPECT_NE(nullptr, wm2);
}

} // namespace
} // namespace alica