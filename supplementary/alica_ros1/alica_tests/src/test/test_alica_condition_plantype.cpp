#include "test_alica.h"

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

class AlicaConditionPlanType : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "MasterPlanTestConditionPlanType"; }
};

/**
 * Test for Runtime or PreCondition are false with plantypes
 */
TEST_F(AlicaConditionPlanType, conditionPlanTypeTest)
{
    ASSERT_NO_SIGNAL

    alica::USummand* uSummandPreConditionPlan = ae->getPlanRepository().getPlans().find(1418042796751)->getUtilityFunction()->getUtilSummands()[0].get();
    alica::TestConstantValueSummand* dbrPre = dynamic_cast<alica::TestConstantValueSummand*>(uSummandPreConditionPlan);
    ASSERT_NE(dbrPre, nullptr);
    dbrPre->robotId = ac->getLocalAgentId();

    alica::USummand* uSummandOtherPlan = ae->getPlanRepository().getPlans().find(1418042819203)->getUtilityFunction()->getUtilSummands()[0].get();
    alica::TestConstantValueSummand* dbrOther = dynamic_cast<alica::TestConstantValueSummand*>(uSummandOtherPlan);
    ASSERT_NE(dbrOther, nullptr);
    dbrOther->robotId = ac->getLocalAgentId();

    alica::USummand* uSummandRunPlan = ae->getPlanRepository().getPlans().find(1418042806575)->getUtilityFunction()->getUtilSummands()[0].get();
    alica::TestConstantValueSummand* dbrRun = dynamic_cast<alica::TestConstantValueSummand*>(uSummandRunPlan);
    ASSERT_NE(dbrRun, nullptr);
    dbrRun->robotId = ac->getLocalAgentId();
    ae->start();
    alicaTests::TestWorldModel* wm = ac->getWorldModel<alicaTests::TestWorldModel>("worldModel");

    for (int i = 0; i < 21; i++) {
        ac->stepEngine();

        //		if(i > 1)
        //		{
        //			long id =
        //(ae->getPlanBase().getRootNode()->getChildren()[0]->getActiveState()->getId();
        //			string name =
        //(ae->getPlanBase().getRootNode()->getChildren()[0]->getActiveState()->getName();
        //			cout << name << " : " << id  << " Iteration : " << i << endl;
        //		}
        if (i == 2) {
            // Should be OtherPlan --> State
            EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1418042819204));
        }
        if (i == 5) {
            wm->setRuntimeCondition1418042967134(true);
        }
        if (i == 6) {
            // Should be RunTimeCondition --> State
            EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1418042806576));
        }
        if (i == 10) {
            wm->setRuntimeCondition1418042967134(false);
        }
        if (i == 12) {
            // Should be OtherPlan --> State
            EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1418042819204));
        }
        if (i == 13) {
            wm->setPreCondition1418042929966(true);
        }
        if (i > 14) {
            // Should be PreCondition --> State
            EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1418042796752));
        }
    }
}
} // namespace
} // namespace alica
