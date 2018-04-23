#include <gtest/gtest.h>
#include <engine/AlicaEngine.h>
#include <engine/AlicaClock.h>
#include "engine/IAlicaCommunication.h"
#include "engine/model/State.h"
#include "engine/model/Behaviour.h"
#include "engine/PlanRepository.h"
#include "engine/BasicBehaviour.h"
#include "engine/BehaviourPool.h"
#include "engine/PlanBase.h"
#include <communication/AlicaRosCommunication.h>
#include "engine/DefaultUtilityFunction.h"
#include "engine/TeamObserver.h"
#include "engine/model/Plan.h"
#include "engine/model/RuntimeCondition.h"
#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "UtilityFunctionCreator.h"
#include "engine/Assignment.h"
#include "engine/collections/AssignmentCollection.h"
#include "engine/collections/StateCollection.h"
#include "Plans/GSolver/SolverTestBehaviour.h"
#include <thread>
#include <iostream>
#include <CGSolver.h>
#include <engine/constraintmodul/Query.h>
#include "ConstraintTestPlanDummySolver.h"
#include <csignal>


class AlicaGSolverPlan : public ::testing::Test {
protected:
    supplementary::SystemConfig* sc;
    alica::AlicaEngine* ae;
    alica::BehaviourCreator* bc;
    alica::ConditionCreator* cc;
    alica::UtilityFunctionCreator* uc;
    alica::ConstraintCreator* crc;

    static void signal_handler(int signal) { EXPECT_FALSE(signal); }
    
    virtual void SetUp() {
        std::signal(SIGSEGV, signal_handler);

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
        ae = new alica::AlicaEngine(new supplementary::AgentIDManager(new supplementary::AgentIDFactory()), "Roleset",
                "GSolverMaster", ".", true);
        bc = new alica::BehaviourCreator();
        cc = new alica::ConditionCreator();
        uc = new alica::UtilityFunctionCreator();
        crc = new alica::ConstraintCreator();
        ae->setAlicaClock(new alica::AlicaClock());
        ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
        ae->addSolver(new alica::reasoner::ConstraintTestPlanDummySolver(ae));
        ae->addSolver(new alica::reasoner::CGSolver(ae));
    }

    virtual void TearDown() {
        ae->shutdown();
        sc->shutdown();
        delete ae->getCommunicator();
        delete ae->getSolver<alica::reasoner::ConstraintTestPlanDummySolver>();
        delete ae->getSolver<alica::reasoner::CGSolver>();
        delete cc;
        delete bc;
        delete uc;
        delete crc;
    }

    static void step(alica::AlicaEngine* ae) {
        ae->stepNotify();
        do {
            ae->getAlicaClock()->sleep(AlicaTime::milliseconds(33));
        } while (!ae->getPlanBase()->isWaiting());
    }
};
/**
 * Tests if Behaviour with Constraints are called
 */
TEST_F(AlicaGSolverPlan, solverTest) {
    ae->init(bc, cc, uc, crc);
    cout << "Starting engine..." << endl;
    ae->start();

    step(ae);

    ASSERT_EQ(alica::SolverTestBehaviour::result.size(), 2) << "Wrong result size";
    EXPECT_GT(alica::SolverTestBehaviour::result[0], 4000);
    EXPECT_LT(alica::SolverTestBehaviour::result[0], 5000);
    EXPECT_GT(alica::SolverTestBehaviour::result[1], 7000);
    EXPECT_LT(alica::SolverTestBehaviour::result[1], 8000);
}
