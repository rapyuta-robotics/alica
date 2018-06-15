#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "ConstraintTestPlanDummySolver.h"
#include "CounterClass.h"
#include "Plans/Behaviour/ConstraintUsingBehaviour.h"
#include "UtilityFunctionCreator.h"
#include "engine/Assignment.h"
#include "engine/BasicBehaviour.h"
#include "engine/BehaviourPool.h"
#include "engine/DefaultUtilityFunction.h"
#include "engine/PlanBase.h"
#include "engine/PlanRepository.h"
#include "engine/TeamObserver.h"
#include "engine/model/Behaviour.h"
#include "engine/model/BehaviourConfiguration.h"
#include "engine/model/Plan.h"
#include "engine/model/RuntimeCondition.h"
#include "engine/model/State.h"
#include <CGSolver.h>
#include <Plans/Behaviour/Attack.h>
#include <communication/AlicaRosCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/constraintmodul/Query.h>
#include <gtest/gtest.h>
#include <iostream>
#include <test_alica.h>
#include <thread>

class AlicaConditionPlan : public ::testing::Test
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
        sc->setHostname("nase");

        // setup the engine
        ae = new alica::AlicaEngine(new supplementary::AgentIDManager(new supplementary::AgentIDFactory()), "Roleset", "ConstraintTestPlan", ".", true);
        bc = new alica::BehaviourCreator();
        cc = new alica::ConditionCreator();
        uc = new alica::UtilityFunctionCreator();
        crc = new alica::ConstraintCreator();
        ae->setAlicaClock(new alica::AlicaClock());
        ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
        ae->addSolver(new alica::reasoner::ConstraintTestPlanDummySolver(ae));
        ae->addSolver(new alica::reasoner::CGSolver(ae));
    }

    virtual void TearDown()
    {
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
};
/**
 * Tests if Behaviour with Constraints are called
 */
TEST_F(AlicaConditionPlan, solverTest)
{
    ASSERT_NO_SIGNAL

    ae->init(bc, cc, uc, crc);

    const alica::PlanRepository* rep = ae->getPlanRepository();

    const alica::BehaviourConfiguration* beh = rep->getBehaviourConfigurations()[1414068618837];
    ASSERT_NE(beh, nullptr);
    const alica::State* state = rep->getStates()[1414068524246];
    ASSERT_NE(state, nullptr);

    ASSERT_EQ(beh->getVariables().size(), 2);
    ASSERT_EQ(state->getParametrisation().size(), 2);
    const alica::Variable* beh_y = nullptr;
    for (const alica::Variable* v : beh->getVariables()) {
        if (v->getName() == "Y") {
            beh_y = v;
            break;
        }
    }
    ASSERT_NE(beh_y, nullptr);

    ASSERT_EQ(beh_y->getId(), 1416488161203);
    bool found = false;
    for (const alica::Parametrisation* p : state->getParametrisation()) {
        ASSERT_EQ(p->getSubPlan(), beh);
        if (p->getSubVar() == beh_y) {
            found = true;
        }
    }
    ASSERT_TRUE(found) << "Sub variable not found in parametrisation";

    ae->start();
    step(ae);

    BasicBehaviour* basicBehaviour = ae->getPlanBase()->getRootNode()->getChildren()[0]->getBasicBehaviour();
    alica::ConstraintUsingBehaviour* constraintUsingBehaviour = dynamic_cast<alica::ConstraintUsingBehaviour*>(basicBehaviour);
    ASSERT_NE(constraintUsingBehaviour, nullptr);
    ASSERT_GT(constraintUsingBehaviour->getCallCounter(), 0);

    ASSERT_GT(alica::reasoner::ConstraintTestPlanDummySolver::getGetSolutionCallCounter(), 0);
    ASSERT_EQ(alica::ConstraintUsingBehaviour::result.size(), 1) << "Wrong result size";
    const ByteArray& ba = ae->getBlackBoard().getValue(alica::ConstraintUsingBehaviour::result[0]);
    std::string resultingString(reinterpret_cast<const char*>(ba.begin()), ba.size());
    EXPECT_EQ("1414068576620", resultingString); // id of variable at highest level
}
