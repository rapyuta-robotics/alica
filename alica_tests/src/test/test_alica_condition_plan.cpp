#include <gtest/gtest.h>
#include <engine/AlicaEngine.h>
#include <engine/AlicaClock.h>
#include "engine/model/State.h"
#include "engine/model/Behaviour.h"
#include "engine/model/BehaviourConfiguration.h"
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
                "ConstraintTestPlan", ".", true);
        bc = new alica::BehaviourCreator();
        cc = new alica::ConditionCreator();
        uc = new alica::UtilityFunctionCreator();
        crc = new alica::ConstraintCreator();
        ae->setAlicaClock(new alica::AlicaClock());
        ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
        ae->addSolver(SolverType::DUMMYSOLVER, new alica::reasoner::ConstraintTestPlanDummySolver(ae));
        ae->addSolver(SolverType::GRADIENTSOLVER, new alica::reasoner::CGSolver(ae));
    }

    virtual void TearDown() {
        ae->shutdown();
        sc->shutdown();
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

    const alica::PlanRepository* rep = ae->getPlanRepository();

    const alica::BehaviourConfiguration* beh = rep->getBehaviourConfigurations()[1414068618837];
    ASSERT_NE(beh, nullptr);
    const alica::State* state = rep->getStates()[1414068524246];
    ASSERT_NE(state, nullptr);

    ASSERT_EQ(beh->getVariables().size(),2);
    ASSERT_EQ(state->getParametrisation().size(),2);
    const alica::Variable* beh_y = nullptr;
    for(const alica::Variable* v : beh->getVariables()) {
        if(v->getName() == "Y") {
            beh_y = v;
            break;
        }
    }
    ASSERT_NE(beh_y, nullptr);

    ASSERT_EQ(beh_y->getId(),1416488161203);
    bool found = false;
    for(const alica::Parametrisation* p : state->getParametrisation()) {
        ASSERT_EQ(p->getSubPlan(), beh);
        if(p->getSubVar() == beh_y) {
            found = true;
        }
    }
    ASSERT_TRUE(found) << "Sub variable not found in parametrisation";

    ae->start();
    ae->stepNotify();
    //	unsigned int sleepTime = 1;
    chrono::milliseconds sleepTime(33);
    this_thread::sleep_for(sleepTime);
    while (!ae->getPlanBase()->isWaiting()) {
        this_thread::sleep_for(sleepTime);
    }

    shared_ptr<BasicBehaviour> basicBehaviour =
            (*ae->getPlanBase()->getRootNode()->getChildren()->begin())->getBasicBehaviour();
    shared_ptr<alica::ConstraintUsingBehaviour> constraintUsingBehaviour = dynamic_pointer_cast<alica::ConstraintUsingBehaviour>(basicBehaviour);
    ASSERT_GT(constraintUsingBehaviour->getCallCounter(), 0);

    ASSERT_GT(alica::reasoner::ConstraintTestPlanDummySolver::getGetSolutionCallCounter(), 0);
    ASSERT_EQ(alica::ConstraintUsingBehaviour::result.size(), 1) << "Wrong result size";
    EXPECT_EQ(alica::ConstraintUsingBehaviour::result[0], "Y");
}
