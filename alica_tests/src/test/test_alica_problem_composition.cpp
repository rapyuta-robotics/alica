#include <BehaviourCreator.h>
#include <CGSolver.h>
#include <ConditionCreator.h>
#include <ConstraintCreator.h>
#include <ConstraintTestPlanDummySolver.h>
#include <FileSystem.h>
#include <Plans/ProblemModule/QueryBehaviour1.h>
#include <SystemConfig.h>
#include <UtilityFunctionCreator.h>
#include <chrono>
#include <communication/AlicaRosCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/PlanBase.h>
#include <engine/RunningPlan.h>
#include <engine/constraintmodul/Query.h>
#include <engine/model/Variable.h>
#include <gtest/gtest.h>
#include <gtest/internal/gtest-internal.h>
#include <iostream>
#include <memory>
#include <string>
#include <test_alica.h>
#include <thread>
#include <vector>

class AlicaProblemCompositionTest : public ::testing::Test
{
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

        // setup the engine
        bc = new alica::BehaviourCreator();
        cc = new alica::ConditionCreator();
        uc = new alica::UtilityFunctionCreator();
        crc = new alica::ConstraintCreator();

        sc->setHostname("nase");
        ae = new alica::AlicaEngine(new supplementary::AgentIDManager(new supplementary::AgentIDFactory()), "Roleset", "ProblemBuildingMaster", true);
        ae->setAlicaClock(new alica::AlicaClock());
        ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
        ae->addSolver(new alica::reasoner::ConstraintTestPlanDummySolver(ae));
        ae->addSolver(new alica::reasoner::CGSolver(ae));
        ae->init(bc, cc, uc, crc);
    }

    virtual void TearDown()
    {
        ae->shutdown();
        delete ae->getCommunicator();
        delete ae->getSolver<alica::reasoner::ConstraintTestPlanDummySolver>();
        delete ae->getSolver<alica::reasoner::CGSolver>();
        sc->shutdown();
        delete cc;
        delete bc;
        delete uc;
        delete crc;
    }
};

/**
 * Tests if static variables and binded correctly.
 */
TEST_F(AlicaProblemCompositionTest, SimpleStaticComposition)
{
    ASSERT_NO_SIGNAL

    ae->start();

    for (int i = 0; i < 6; ++i) {
        step(ae);
    }

    const RunningPlan* deep = ae->getPlanBase()->getDeepestNode();

    ASSERT_FALSE(deep == nullptr);
    ASSERT_EQ(deep->getChildren().size(), 1);
    ASSERT_TRUE((*deep->getChildren().begin())->isBehaviour());

    auto queryBehaviour1 = dynamic_cast<QueryBehaviour1*>((*deep->getChildren().begin())->getBasicBehaviour());
    ASSERT_NE(queryBehaviour1, nullptr);
    VariableGrp allReps;
    queryBehaviour1->query->getUniqueVariableStore().getAllRep(allReps);

    for (const alica::Variable* rep : allReps) {
        EXPECT_TRUE(rep->getName() == "PBMX" || rep->getName() == "PBMY");
        // cout << "Test: '" << rep->getName() << "'" << endl;
    }
}
