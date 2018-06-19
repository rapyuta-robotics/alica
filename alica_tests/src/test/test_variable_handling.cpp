#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "Plans/VariableHandling/Lvl11524452759599.h"
#include "UtilityFunctionCreator.h"
#include "engine/Assignment.h"
#include "engine/IAlicaCommunication.h"
#include "engine/PlanBase.h"
#include "engine/TeamObserver.h"

#include "engine/model/Plan.h"
#include "engine/model/State.h"
#include <CGSolver.h>
#include <communication/AlicaRosCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/constraintmodul/Query.h>
#include <gtest/gtest.h>
#include <iostream>
#include <thread>

using alica::reasoner::CGSolver;

class AlicaVariableHandlingTest : public ::testing::Test
{
protected:
    supplementary::SystemConfig* sc;
    alica::AlicaEngine* ae1;
    alica::AlicaEngine* ae2;
    alica::BehaviourCreator* bc1;
    alica::ConditionCreator* cc1;
    alica::UtilityFunctionCreator* uc1;
    alica::ConstraintCreator* crc1;

    alica::BehaviourCreator* bc2;
    alica::ConditionCreator* cc2;
    alica::UtilityFunctionCreator* uc2;
    alica::ConstraintCreator* crc2;

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
        ae1 = new alica::AlicaEngine(new supplementary::AgentIDManager(new supplementary::AgentIDFactory()), "RolesetTA", "VHMaster", ".", true);
        bc1 = new alica::BehaviourCreator();
        cc1 = new alica::ConditionCreator();
        uc1 = new alica::UtilityFunctionCreator();
        crc1 = new alica::ConstraintCreator();
        ae1->setAlicaClock(new alica::AlicaClock());
        ae1->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae1));
        ae1->addSolver(new CGSolver(ae1));

        sc->setHostname("hairy");

        ae2 = new alica::AlicaEngine(new supplementary::AgentIDManager(new supplementary::AgentIDFactory()), "RolesetTA", "VHMaster", ".", true);
        bc2 = new alica::BehaviourCreator();
        cc2 = new alica::ConditionCreator();
        uc2 = new alica::UtilityFunctionCreator();
        crc2 = new alica::ConstraintCreator();
        ae2->setAlicaClock(new alica::AlicaClock());
        ae2->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae2));
        ae2->addSolver(new CGSolver(ae2));
    }

    virtual void TearDown()
    {
        ae1->shutdown();
        ae2->shutdown();

        delete ae1->getCommunicator();
        delete ae1->getSolver<CGSolver>();
        delete crc1;
        delete uc1;
        delete cc1;
        delete bc1;

        delete ae2->getCommunicator();
        delete ae2->getSolver<CGSolver>();
        delete crc2;
        delete uc2;
        delete cc2;
        delete bc2;

        sc->shutdown();
    }
};

TEST_F(AlicaVariableHandlingTest, testQueries)
{
    ae1->init(bc1, cc1, uc1, crc1);
    ae2->init(bc2, cc2, uc2, crc2);
    cout << "Starting engine..." << endl;
    ae1->start();
    ae2->start();

    chrono::milliseconds sleepTime(33);
    do {
        for (int i = 0; i < 3; ++i) {
            ae1->stepNotify();
            ae2->stepNotify();
            do {
                this_thread::sleep_for(sleepTime);
            } while (!ae1->getPlanBase()->isWaiting() || !ae2->getPlanBase()->isWaiting());
        }
    } while (ae1->getTeamManager()->getTeamSize() != 2 || ae2->getTeamManager()->getTeamSize() != 2);

    const RunningPlan* rp1 = ae1->getPlanBase()->getDeepestNode();
    const RunningPlan* rp2 = ae1->getPlanBase()->getDeepestNode();

    alica::AgentIDConstPtr id1 = ae1->getTeamManager()->getLocalAgentID();
    alica::AgentIDConstPtr id2 = ae2->getTeamManager()->getLocalAgentID();
    EXPECT_NE(*id1, *id2) << "Agents use the same ID.";

    EXPECT_EQ(rp1->getActivePlan()->getId(), rp2->getActivePlan()->getId());
    EXPECT_EQ(rp1->getActivePlan()->getName(), "Lvl1");
    EXPECT_EQ(rp1->getAssignment().size(), 2u);
    EXPECT_EQ(rp2->getAssignment().size(), 2u);

    EXPECT_EQ(rp1->getActiveState()->getId(), 1524453481856);
    EXPECT_EQ(rp2->getActiveState()->getId(), 1524453481856);

    std::vector<double> result1;
    bool ok;

    alica::Query q1;

    q1.addDomainVariable(id1, "X", ae1);
    ok = q1.getSolution<CGSolver, double>(ThreadSafePlanInterface(rp1), result1);
    EXPECT_FALSE(ok);
    EXPECT_TRUE(result1.empty());
    EXPECT_EQ(0, q1.getPartCount());

    q1.clearDomainVariables();

    const alica::Variable* v1 = rp1->getActivePlan()->getVariableByName("L1A");
    const alica::Variable* v2 = rp1->getActivePlan()->getVariableByName("L1B");

    EXPECT_NE(v1, nullptr);
    EXPECT_NE(v2, nullptr);

    q1.addStaticVariable(v1);
    q1.addStaticVariable(v2);

    ok = q1.getSolution<CGSolver, double>(ThreadSafePlanInterface(rp1), result1);
    EXPECT_TRUE(ok);
    EXPECT_EQ(2, result1.size());
    EXPECT_EQ(1, q1.getPartCount());
    EXPECT_LT(result1[0], 0);
    EXPECT_GT(result1[1], 0);
    EXPECT_LT(result1[0] + result1[1], 10.0);
    EXPECT_GT(result1[0] + result1[1], -10.0);
    q1.addDomainVariable(id1, "X", ae1);
    ok = q1.getSolution<CGSolver, double>(ThreadSafePlanInterface(rp1), result1);
    EXPECT_TRUE(ok);
    EXPECT_EQ(3, result1.size());
    EXPECT_EQ(1, q1.getPartCount());

    // Cause  agent to move through transition:
    vhStartCondition = true;
    ae1->stepNotify();
    ae2->stepNotify();
    do {
        this_thread::sleep_for(sleepTime);
    } while (!ae1->getPlanBase()->isWaiting() || !ae2->getPlanBase()->isWaiting());

    rp1 = ae1->getPlanBase()->getDeepestNode();
    rp2 = ae2->getPlanBase()->getDeepestNode();

    EXPECT_EQ(1524452836023, rp1->getActiveState()->getId()); // lvl3
    EXPECT_EQ(1524453248579, rp2->getActiveState()->getId()); // Dummy in lvl2

    q1.clearStaticVariables();
    q1.clearDomainVariables();

    v1 = rp1->getActivePlan()->getVariableByName("L3A");
    v2 = rp1->getActivePlan()->getVariableByName("L3B");
    q1.addStaticVariable(v1);
    q1.addStaticVariable(v2);

    ok = q1.getSolution<CGSolver, double>(ThreadSafePlanInterface(rp1), result1);
    EXPECT_TRUE(ok);
    EXPECT_EQ(2, result1.size());
    EXPECT_EQ(4, q1.getPartCount());

    Query q2;
    q2.addDomainVariable(id2, "X", ae2);
    q2.addDomainVariable(id2, "Y", ae2);

    ok = q2.getSolution<CGSolver, double>(ThreadSafePlanInterface(rp2), result1);
    EXPECT_TRUE(ok);
    EXPECT_EQ(2, result1.size());
    EXPECT_EQ(3, q2.getPartCount());
    EXPECT_GT(result1[0] + 0.001, result1[1]);

    q1.clearStaticVariables();
    q1.clearDomainVariables();

    q1.addDomainVariable(id1, "X", ae1);
    q1.addDomainVariable(id1, "Y", ae1);
    ok = q1.getSolution<CGSolver, double>(ThreadSafePlanInterface(rp1), result1);
    EXPECT_TRUE(ok);
    EXPECT_EQ(2, result1.size());
    EXPECT_EQ(4, q1.getPartCount());
    EXPECT_GT(result1[0] + 0.001, result1[1]);
}
