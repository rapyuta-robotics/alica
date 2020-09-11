#pragma once

#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "ConstraintTestPlanDummySolver.h"
#include "UtilityFunctionCreator.h"
#include <TestWorldModel.h>

#include <alica/test/TestContext.h>
#include <communication/AlicaDummyCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaContext.h>
#include <engine/AlicaEngine.h>

#include <csetjmp>
#include <csignal>
#include <string>

#include <ros/ros.h>

#include <gtest/gtest.h>

#define ASSERT_NO_SIGNAL ASSERT_EQ(setjmp(restore_point), 0);

namespace alica
{

/**
 * This Getter-Struct provides access to the engine for alica
 * tests. Application tests however should not have access to the
 * engine directly, but must live with the API of the
 * Alica TestContext class.
 */
struct AlicaTestsEngineGetter
{
    static alica::AlicaEngine* getEngine(alica::test::TestContext* tc) { return tc->_engine.get(); }
};

class AlicaTestFixtureBase : public ::testing::Test
{
protected:
    alica::test::TestContext* tc;
};

class AlicaTestFixture : public AlicaTestFixtureBase
{
protected:
    virtual const char* getRoleSetName() const { return "Roleset"; }
    virtual const char* getMasterPlanName() const = 0;
    virtual bool stepEngine() const { return true; }
    void SetUp() override
    {
        alicaTests::TestWorldModel::getOne()->reset();
        alicaTests::TestWorldModel::getTwo()->reset();

        // determine the path to the test config
        ros::NodeHandle nh;
        std::string path;
        nh.param<std::string>("/rootPath", path, ".");
        alica::AlicaContext::setLocalAgentName("nase");
        alica::AlicaContext::setRootPath(path);
        alica::AlicaContext::setConfigPath(path + "/etc");
        tc = new alica::test::TestContext(getRoleSetName(), getMasterPlanName(), stepEngine());
        ASSERT_TRUE(tc->isValid());
        tc->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
        alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>());
        EXPECT_EQ(tc->init(creators), 0);
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
    }
    void TearDown() override { AlicaTestFixture::TearDown(); }
};

class AlicaTestMultiAgentFixtureBase : public ::testing::Test
{
protected:
    std::vector<alica::test::TestContext*> tcs;
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
        alicaTests::TestWorldModel::getOne()->reset();
        alicaTests::TestWorldModel::getTwo()->reset();
        // determine the path to the test config
        ros::NodeHandle nh;
        std::string path;
        nh.param<std::string>("/rootPath", path, ".");
        alica::AlicaContext::setRootPath(path);
        alica::AlicaContext::setConfigPath(path + "/etc");
        alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>());

        for (int i = 0; i < getAgentCount(); ++i) {
            alica::test::TestContext::setLocalAgentName(getHostName(i));
            alica::test::TestContext* tc = new alica::test::TestContext(getRoleSetName(), getMasterPlanName(), stepEngine());
            ASSERT_TRUE(tc->isValid());
            tc->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
            EXPECT_EQ(tc->init(creators), 0);
            tcs.push_back(tc);
        }
    }

    void TearDown() override
    {
        for (alica::AlicaContext* ac : tcs) {
            ac->terminate();
            delete ac;
        }
    }
};
} // namespace alica

extern std::jmp_buf restore_point;
void signalHandler(int signal);
