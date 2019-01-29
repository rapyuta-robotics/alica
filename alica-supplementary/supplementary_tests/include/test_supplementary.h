#pragma once

#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "ConstraintTestPlanDummySolver.h"
#include "UtilityFunctionCreator.h"

#include <CGSolver.h>
#include <SystemConfig.h>
#include <communication/AlicaDummyCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>

#include <csetjmp>
#include <csignal>
#include <string>

#include <ros/ros.h>

#include <gtest/gtest.h>

#define ASSERT_NO_SIGNAL ASSERT_EQ(setjmp(restore_point), 0);

class AlicaTestFixtureBase : public ::testing::Test
{
protected:
    essentials::SystemConfig* sc;
    alica::AlicaEngine* ae;
    alica::BehaviourCreator* bc;
    alica::ConditionCreator* cc;
    alica::UtilityFunctionCreator* uc;
    alica::ConstraintCreator* crc;
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

        // bring up the SystemConfig with the corresponding path
        sc = essentials::SystemConfig::getInstance();
        sc->setRootPath(path);
        sc->setConfigPath(path + "/etc");
        sc->setHostname("nase");

        // setup the engine
        ae = new alica::AlicaEngine(
                new essentials::AgentIDManager(new essentials::AgentIDFactory()), getRoleSetName(), getMasterPlanName(), stepEngine());
        bc = new alica::BehaviourCreator();
        cc = new alica::ConditionCreator();
        uc = new alica::UtilityFunctionCreator();
        crc = new alica::ConstraintCreator();
        ae->setAlicaClock(new alica::AlicaClock());
        ae->setCommunicator(new alicaDummyProxy::AlicaDummyCommunication(ae));

        EXPECT_TRUE(ae->init(bc, cc, uc, crc));
    }

    void TearDown() override
    {
        ae->shutdown();
        sc->shutdown();
        delete ae->getCommunicator();
        delete cc;
        delete bc;
        delete uc;
        delete crc;
    }
};

class AlicaTestFixtureWithSolvers : public AlicaTestFixture
{
protected:
    void SetUp() override
    {
        AlicaTestFixture::SetUp();
        ae->addSolver(new alica::reasoner::ConstraintTestPlanDummySolver(ae));
        ae->addSolver(new alica::reasoner::CGSolver(ae));
    }
    void TearDown() override
    {
        alica::ISolverBase* s1 = ae->getSolver<alica::reasoner::ConstraintTestPlanDummySolver>();
        alica::ISolverBase* s2 = ae->getSolver<alica::reasoner::CGSolver>();
        AlicaTestFixture::TearDown();
        delete s1;
        delete s2;
    }
};

extern std::jmp_buf restore_point;
void signalHandler(int signal);
void step(alica::AlicaEngine* ae);
