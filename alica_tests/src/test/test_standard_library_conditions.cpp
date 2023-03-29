#include "test_alica.h"

#include <alica/test/CounterClass.h>
#include <alica_tests/TestConstantValueSummand.h>
#include <alica_tests/TestWorldModel.h>

#include <alica/test/Util.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/IAlicaCommunication.h>
#include <engine/PlanBase.h>
#include <engine/PlanRepository.h>
#include <engine/TeamObserver.h>
#include <engine/UtilityFunction.h>
#include <engine/model/Plan.h>
#include <engine/model/State.h>
#include <engine/teammanager/TeamManager.h>

#include <gtest/gtest.h>

namespace alica
{
namespace
{

class StandardLibraryCompareConditions : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "TestStandardLibraryCompareConditions"; }
};

/*
    Test that the standard library compare conditions like IsEqualTo, IsNotEqualTo, IsGreaterThan, etc work as expected.
*/
TEST_F(StandardLibraryCompareConditions, comparConditions)
{
    ae->start();
    std::shared_ptr<alicaTests::TestWorldModel> wm =
            LockedBlackboardRW(ac->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");

    STEP_UNTIL(alica::test::Util::isStateActive(ae, 3632801511049529612));
}
} // namespace
} // namespace alica
