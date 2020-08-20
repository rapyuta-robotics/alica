
#include "VariableHandling/Lvl11524452759599.h"
#include <test_supplementary.h>

#include "engine/Assignment.h"
#include "engine/PlanBase.h"
#include "engine/TeamObserver.h"
#include "engine/model/Plan.h"
#include "engine/model/State.h"
#include <constraintsolver/CGSolver.h>
#include <communication/AlicaRosCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/constraintmodul/Query.h>
#include <gtest/gtest.h>
#include <iostream>
#include <thread>

namespace supplementary
{
namespace
{

using alica::reasoner::CGSolver;

class AlicaVariableHandlingTest : public AlicaTestMultiAgentFixture
{
protected:
    const int agentCount = 2;
    const char* getRoleSetName() const override { return "RolesetTA"; }
    const char* getMasterPlanName() const override { return "VHMaster"; }
    int getAgentCount() const override { return agentCount; }
    const char* getHostName(int agentNumber) const override
    {
        if (agentNumber) {
            return "hairy";
        } else {
            return "nase";
        }
    }

    virtual void SetUp()
    {
        AlicaTestMultiAgentFixture::SetUp();
        tcs[0]->addSolver<alica::reasoner::CGSolver>();
        tcs[1]->addSolver<alica::reasoner::CGSolver>();
    }
};

TEST_F(AlicaVariableHandlingTest, testQueries)
{
    std::cout << "Starting engine..." << std::endl;
    aes[0]->start();
    aes[1]->start();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());

    std::chrono::milliseconds sleepTime(33);
    do {
        for (int i = 0; i < 3; ++i) {
            step(aes[0]);
            step(aes[1]);
        }
    } while (aes[0]->getTeamManager().getTeamSize() != 2 || aes[1]->getTeamManager().getTeamSize() != 2);

    const RunningPlan* rp1 = aes[0]->getPlanBase().getDeepestNode();
    const RunningPlan* rp2 = aes[0]->getPlanBase().getDeepestNode();

    essentials::IdentifierConstPtr id1 = aes[0]->getTeamManager().getLocalAgentID();
    essentials::IdentifierConstPtr id2 = aes[1]->getTeamManager().getLocalAgentID();
    EXPECT_NE(id1, id2) << "Agents use the same ID.";

    EXPECT_EQ(rp1->getActivePlan()->getId(), rp2->getActivePlan()->getId());
    EXPECT_EQ(rp1->getActivePlan()->getName(), "Lvl1");
    EXPECT_EQ(rp1->getAssignment().size(), 2u);
    EXPECT_EQ(rp2->getAssignment().size(), 2u);

    EXPECT_EQ(rp1->getActiveState()->getId(), 1524453481856);
    EXPECT_EQ(rp2->getActiveState()->getId(), 1524453481856);

    std::vector<double> result1;
    bool ok;

    alica::Query q1;

    q1.addDomainVariable(id1, "X", aes[0]);
    ok = q1.getSolution<CGSolver, double>(ThreadSafePlanInterface(rp1), result1);
    EXPECT_FALSE(ok);
    EXPECT_TRUE(result1.empty());
    EXPECT_EQ(0, q1.getPartCount());

    q1.clearDomainVariables();

    const alica::Variable* v1 = rp1->getActivePlan()->getVariable("L1A");
    const alica::Variable* v2 = rp1->getActivePlan()->getVariable("L1B");

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
    q1.addDomainVariable(id1, "X", aes[0]);
    ok = q1.getSolution<CGSolver, double>(ThreadSafePlanInterface(rp1), result1);
    EXPECT_TRUE(ok);
    EXPECT_EQ(3, result1.size());
    EXPECT_EQ(1, q1.getPartCount());

    // Cause  agent to move through transition:
    vhStartCondition = true;
    aes[0]->stepNotify();
    aes[1]->stepNotify();
    do {
        std::this_thread::sleep_for(sleepTime);
    } while (!aes[0]->getPlanBase().isWaiting() || !aes[1]->getPlanBase().isWaiting());

    rp1 = aes[0]->getPlanBase().getDeepestNode();
    rp2 = aes[1]->getPlanBase().getDeepestNode();

    EXPECT_EQ(1524452836023, rp1->getActiveState()->getId()); // lvl3
    EXPECT_EQ(1524453248579, rp2->getActiveState()->getId()); // Dummy in lvl2

    q1.clearStaticVariables();
    q1.clearDomainVariables();

    v1 = rp1->getActivePlan()->getVariable("L3A");
    v2 = rp1->getActivePlan()->getVariable("L3B");
    q1.addStaticVariable(v1);
    q1.addStaticVariable(v2);

    ok = q1.getSolution<CGSolver, double>(ThreadSafePlanInterface(rp1), result1);
    EXPECT_TRUE(ok);
    EXPECT_EQ(2, result1.size());
    EXPECT_EQ(4, q1.getPartCount());

    Query q2;
    q2.addDomainVariable(id2, "X", aes[1]);
    q2.addDomainVariable(id2, "Y", aes[1]);

    ok = q2.getSolution<CGSolver, double>(ThreadSafePlanInterface(rp2), result1);
    EXPECT_TRUE(ok);
    EXPECT_EQ(2, result1.size());
    EXPECT_EQ(3, q2.getPartCount());
    EXPECT_GT(result1[0] + 0.001, result1[1]);

    q1.clearStaticVariables();
    q1.clearDomainVariables();

    q1.addDomainVariable(id1, "X", aes[0]);
    q1.addDomainVariable(id1, "Y", aes[0]);
    ok = q1.getSolution<CGSolver, double>(ThreadSafePlanInterface(rp1), result1);
    EXPECT_TRUE(ok);
    EXPECT_EQ(2, result1.size());
    EXPECT_EQ(4, q1.getPartCount());
    EXPECT_GT(result1[0] + 0.001, result1[1]);
}
}
}