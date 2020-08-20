#pragma once

#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "ConstraintTestPlanDummySolver.h"
#include "UtilityFunctionCreator.h"

#include "communication/AlicaRosCommunication.h"
#include <constraintsolver/CGSolver.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaContext.h>
#include <engine/AlicaEngine.h>
#include <alica/test/TestContext.h>

#include <csetjmp>
#include <csignal>
#include <string>

#include <ros/ros.h>

#include <gtest/gtest.h>

#define ASSERT_NO_SIGNAL ASSERT_EQ(setjmp(restore_point), 0);

namespace supplementary
{

class AlicaTestFixtureBase : public ::testing::Test
{
protected:
    alica::test::TestContext* tc;
    alica::AlicaEngine* ae;
};

class AlicaTestFixture : public AlicaTestFixtureBase
{
protected:
    virtual const char* getRoleSetName() const { return "Roleset"; }
    virtual const char* getMasterPlanName() const = 0;
    virtual bool stepEngine() const { return true; }
    void SetUp() override
    {
        // determine the path to the test config
        ros::NodeHandle nh;
        std::string path;
        nh.param<std::string>("/rootPath", path, ".");
        alica::AlicaContext::setLocalAgentName("nase");
        alica::AlicaContext::setRootPath(path);
        alica::AlicaContext::setConfigPath(path + "/etc");
        tc = new alica::test::TestContext(getRoleSetName(), getMasterPlanName(), stepEngine());
        ASSERT_TRUE(tc->isValid());
        tc->setCommunicator<alicaRosProxy::AlicaRosCommunication>();

        ae = tc->getEngine();
        alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>());
        // Applications are supposed to call AlicaContext::init() api. Unit tests on the other hand need finer control.
        // ac->init(creators);
        const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
        EXPECT_TRUE(ae->init(creators));
    }

    void TearDown() override
    {
        tc->terminate();
        delete tc;
    }
};

class AlicaTestFixtureWithSolvers : public AlicaTestFixture
{
protected:
    void SetUp() override
    {
        AlicaTestFixture::SetUp();
        tc->addSolver<alica::reasoner::ConstraintTestPlanDummySolver>();
        tc->addSolver<alica::reasoner::CGSolver>();
    }
    void TearDown() override { AlicaTestFixture::TearDown(); }
};

class AlicaTestMultiAgentFixtureBase : public ::testing::Test
{
protected:
    std::vector<alica::test::TestContext*> tcs;
    std::vector<alica::AlicaEngine*> aes;
};

class AlicaTestMultiAgentFixture : public AlicaTestMultiAgentFixtureBase
{
protected:
    virtual const char* getRoleSetName() const { return "Roleset"; }
    virtual const char* getMasterPlanName() const = 0;
    virtual int getAgentCount() const = 0;
    virtual bool stepEngine() const { return true; }
    virtual const char* getHostName(int agentNumber) const { return "nase"; }
    virtual alica::AlicaTime getDiscoveryTimeout() const { return alica::AlicaTime::milliseconds(100); }

    void SetUp() override
    {
        // determine the path to the test config
        ros::NodeHandle nh;
        std::string path;
        nh.param<std::string>("/rootPath", path, ".");
        alica::AlicaContext::setRootPath(path);
        alica::AlicaContext::setConfigPath(path + "/etc");
        alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>());

        for (int i = 0; i < getAgentCount(); ++i) {
            alica::AlicaContext::setLocalAgentName(getHostName(i));
            alica::test::TestContext* tc = new alica::test::TestContext(getRoleSetName(), getMasterPlanName(), stepEngine());
            ASSERT_TRUE(tc->isValid());
            tc->setCommunicator<alicaRosProxy::AlicaRosCommunication>();
            alica::AlicaEngine* ae = tc->getEngine();
            const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
            EXPECT_TRUE(ae->init(creators));
            tcs.push_back(tc);
            aes.push_back(ae);
        }
    }

    void TearDown() override
    {
        for (alica::AlicaContext* tc : tcs) {
            tc->terminate();
            delete tc;
        }
    }
};
}
extern std::jmp_buf restore_point;
void signalHandler(int signal);
void step(alica::AlicaEngine* ae);
