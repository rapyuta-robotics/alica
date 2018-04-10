#include <gtest/gtest.h>
#include <engine/AlicaEngine.h>
#include <engine/AlicaClock.h>
#include "engine/model/State.h"
#include "engine/model/Behaviour.h"
#include "engine/PlanRepository.h"
#include "engine/BasicBehaviour.h"
#include "engine/BehaviourPool.h"
#include "engine/PlanBase.h"
#include <engine/AlicaClock.h>
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
#include <thread>
#include <iostream>
#include "SolverType.h"
#include <CGSolver.h>
#include <engine/constraintmodul/Query.h>
#include <Plans/Behaviour/Attack.h>
#include "CounterClass.h"
#include "ConstraintTestPlanDummySolver.h"
#include "Plans/Behaviour/ConstraintUsingBehaviour.h"

class AlicaConditionPlan : public ::testing::Test {
protected:
    supplementary::SystemConfig* sc;
    alica::AlicaEngine* ae;
    alica::BehaviourCreator* bc;
    alica::ConditionCreator* cc;
    alica::UtilityFunctionCreator* uc;
    alica::ConstraintCreator* crc;

    virtual void SetUp() {
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
                "ConstraintTestPlan", ".", false);
        bc = new alica::BehaviourCreator();
        cc = new alica::ConditionCreator();
        uc = new alica::UtilityFunctionCreator();
        crc = new alica::ConstraintCreator();
        ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
        ae->addSolver(SolverType::DUMMYSOLVER, new alica::reasoner::ConstraintTestPlanDummySolver(ae));
        ae->addSolver(SolverType::GRADIENTSOLVER, new alica::reasoner::CGSolver(ae));
    }

    virtual void TearDown() {
        ae->shutdown();
        sc->shutdown();
        delete ae->getAlicaClock();
        delete ae->getCommunicator();
        delete ae->getSolver(SolverType::DUMMYSOLVER);
        delete ae->getSolver(SolverType::GRADIENTSOLVER);
        delete cc;
        delete bc;
        delete uc;
        delete crc;
    }
};
/**
 * Tests if Behaviour with Constraints are called
 */
TEST_F(AlicaConditionPlan, solverTest) {
    ae->init(bc, cc, uc, crc);
    ae->start();

    //	unsigned int sleepTime = 1;
    chrono::milliseconds sleepTime(33);
    this_thread::sleep_for(sleepTime);

    //	shared_ptr<BasicBehaviour> basicBehaviour =
    //			(*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getBasicBehaviour();
    //	shared_ptr<alicaTests::ConstraintUsingBehaviour> constraintUsingBehaviour = dynamic_pointer_cast<
    //			alicaTests::ConstraintUsingBehaviour>(basicBehaviour);
    //	EXPECT_GT(constraintUsingBehaviour->getCallCounter(), 0);

    ASSERT_GT(alicaAutogenerated::CounterClass::called, 0);

    ASSERT_GT(alica::reasoner::ConstraintTestPlanDummySolver::getGetSolutionCallCounter(), 0);
    ASSERT_EQ(alica::ConstraintUsingBehaviour::result.size(), 1) << "Wrong result size";
    EXPECT_EQ(alica::ConstraintUsingBehaviour::result[0], "Y");
    alicaAutogenerated::CounterClass::called = 0;
}
