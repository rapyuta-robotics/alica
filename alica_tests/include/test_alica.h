#pragma once

#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "ConstraintTestPlanDummySolver.h"
#include "UtilityFunctionCreator.h"

#include <communication/AlicaDummyCommunication.h>
#include <engine/AlicaContext.h>

#include <csetjmp>
#include <csignal>
#include <string>

#include <ros/ros.h>

#include <gtest/gtest.h>

#define ASSERT_NO_SIGNAL ASSERT_EQ(setjmp(restore_point), 0);

class AlicaTestFixtureBase : public ::testing::Test
{
protected:
    alica::AlicaContext* ac;
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

        ac = new alica::AlicaContext(getRoleSetName(), getMasterPlanName(), stepEngine());
        EXPECT_TRUE(ac->isValid());
        ac->setRobotName("nase");
        ac->setRootPath(path);
        ac->setConfigPath(path + "/etc");

        alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>());

        ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
        EXPECT_TRUE(ac->init(creators));
    }

    void TearDown() override
    {
        ac->terminate();
        delete ac;
    }
};

class AlicaTestFixtureWithSolvers : public AlicaTestFixture
{
protected:
    void SetUp() override
    {
        AlicaTestFixture::SetUp();
        ac->addSolver<alica::reasoner::ConstraintTestPlanDummySolver>();
    }
    void TearDown() override { AlicaTestFixture::TearDown(); }
};

extern std::jmp_buf restore_point;
void signalHandler(int signal);
void step(alica::AlicaEngine* ae);
