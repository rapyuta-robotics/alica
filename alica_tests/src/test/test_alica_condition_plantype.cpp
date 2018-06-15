#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "TestConstantValueSummand.h"
#include "TestWorldModel.h"
#include "UtilityFunctionCreator.h"
#include "engine/IAlicaCommunication.h"
#include "engine/PlanBase.h"
#include "engine/PlanRepository.h"
#include "engine/TeamObserver.h"
#include "engine/UtilityFunction.h"
#include "engine/model/Plan.h"
#include "engine/model/State.h"
#include "engine/teammanager/TeamManager.h"
#include <communication/AlicaRosCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <gtest/gtest.h>
#include <test_alica.h>

class AlicaConditionPlanType : public ::testing::Test
{ /* namespace alicaTests */
protected:
    supplementary::SystemConfig* sc;
    alica::AlicaEngine* ae;
    alica::BehaviourCreator* bc;
    alica::ConditionCreator* cc;
    alica::UtilityFunctionCreator* uc;
    alica::ConstraintCreator* crc;

    virtual void SetUp()
    {
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
        ae = new alica::AlicaEngine(
                new supplementary::AgentIDManager(new supplementary::AgentIDFactory()), "Roleset", "MasterPlanTestConditionPlanType", ".", true);
        bc = new alica::BehaviourCreator();
        cc = new alica::ConditionCreator();
        uc = new alica::UtilityFunctionCreator();
        crc = new alica::ConstraintCreator();
        ae->setAlicaClock(new alica::AlicaClock());
    }

    virtual void TearDown()
    {
        ae->shutdown();
        sc->shutdown();
        delete cc;
        delete bc;
        delete uc;
        delete crc;
    }
};
/**
 * Test for Runtime or PreCondition are false with plantypes
 */
TEST_F(AlicaConditionPlanType, conditionPlanTypeTest)
{
    ASSERT_NO_SIGNAL

    ae->setAlicaClock(new alica::AlicaClock());
    ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
    EXPECT_TRUE(ae->init(bc, cc, uc, crc)) << "Unable to initialise the Alica Engine!";

    USummand* uSummandPreConditionPlan = ae->getPlanRepository()->getPlans().find(1418042796751)->getUtilityFunction()->getUtilSummands()[0].get();
    TestConstantValueSummand* dbrPre = dynamic_cast<TestConstantValueSummand*>(uSummandPreConditionPlan);
    ASSERT_NE(dbrPre, nullptr);
    dbrPre->robotId = ae->getTeamManager()->getLocalAgentID();

    USummand* uSummandOtherPlan = ae->getPlanRepository()->getPlans().find(1418042819203)->getUtilityFunction()->getUtilSummands()[0].get();
    TestConstantValueSummand* dbrOther = dynamic_cast<TestConstantValueSummand*>(uSummandOtherPlan);
    ASSERT_NE(dbrOther, nullptr);
    dbrOther->robotId = ae->getTeamManager()->getLocalAgentID();

    USummand* uSummandRunPlan = ae->getPlanRepository()->getPlans().find(1418042806575)->getUtilityFunction()->getUtilSummands()[0].get();
    TestConstantValueSummand* dbrRun = dynamic_cast<TestConstantValueSummand*>(uSummandRunPlan);
    ASSERT_NE(dbrRun, nullptr);
    dbrRun->robotId = ae->getTeamManager()->getLocalAgentID();

    ae->start();

    for (int i = 0; i < 21; i++) {
        step(ae);

        //		if(i > 1)
        //		{
        //			long id =
        //(ae->getPlanBase()->getRootNode()->getChildren()[0]->getActiveState()->getId();
        //			string name =
        //(ae->getPlanBase()->getRootNode()->getChildren()[0]->getActiveState()->getName();
        //			cout << name << " : " << id  << " Iteration : " << i << endl;
        //		}
        if (i == 2) {
            // Should be OtherPlan --> State
            EXPECT_EQ(ae->getPlanBase()->getRootNode()->getChildren()[0]->getActiveState()->getId(), 1418042819204);
        }
        if (i == 5) {
            alicaTests::TestWorldModel::getOne()->setRuntimeCondition1418042967134(true);
        }
        if (i == 6) {
            // Should be RunTimeCondition --> State
            EXPECT_EQ(ae->getPlanBase()->getRootNode()->getChildren()[0]->getActiveState()->getId(), 1418042806576);
        }
        if (i == 10) {
            alicaTests::TestWorldModel::getOne()->setRuntimeCondition1418042967134(false);
        }
        if (i == 12) {
            // Should be OtherPlan --> State
            EXPECT_EQ(ae->getPlanBase()->getRootNode()->getChildren()[0]->getActiveState()->getId(), 1418042819204);
        }
        if (i == 13) {
            alicaTests::TestWorldModel::getOne()->setPreCondition1418042929966(true);
        }
        if (i > 14) {
            // Should be PreCondition --> State
            EXPECT_EQ(ae->getPlanBase()->getRootNode()->getChildren()[0]->getActiveState()->getId(), 1418042796752);
        }
    }
}
