#pragma once

#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "ConstraintTestPlanDummySolver.h"
#include "PlanCreator.h"
#include "UtilityFunctionCreator.h"

#include "communication/AlicaRosCommunication.h"
#include <clock/AlicaRosTimer.h>
#include <constraintsolver/CGSolver.h>
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
    static alica::AlicaEngine* getEngine(alica::AlicaContext* ac) { return ac->_engine.get(); }
};
} // namespace alica

namespace supplementary
{

class AlicaTestFixtureBase : public ::testing::Test
{
protected:
    alica::AlicaContext* ac;
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
        nh.param<std::string>("rootPath", path, ".");
        ac = new alica::AlicaContext(AlicaContextParams("nase", path + "/etc", getRoleSetName(), getMasterPlanName(), stepEngine()));
        ASSERT_TRUE(ac->isValid());
        spinner = std::make_unique<ros::AsyncSpinner>(4);
        ac->setCommunicator<alicaRosProxy::AlicaRosCommunication>();
        ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>();
        alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(), std::make_unique<alica::PlanCreator>());
        ae = AlicaTestsEngineGetter::getEngine(ac);
        const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
        spinner->start();
        EXPECT_TRUE(ae->init(creators));
    }

    void TearDown() override
    {
        spinner->stop();
        ac->terminate();
        delete ac;
    }

    std::unique_ptr<ros::AsyncSpinner> spinner;
};

class AlicaTestFixtureWithSolvers : public AlicaTestFixture
{
protected:
    void SetUp() override
    {
        AlicaTestFixture::SetUp();
        ac->addSolver<alica::reasoner::ConstraintTestPlanDummySolver>();
        ac->addSolver<alica::reasoner::CGSolver>();
    }
    void TearDown() override { AlicaTestFixture::TearDown(); }
};

class AlicaTestMultiAgentFixtureBase : public ::testing::Test
{
protected:
    std::vector<alica::AlicaContext*> acs;
    std::vector<alica::AlicaEngine*> aes;
    std::vector<std::unique_ptr<ros::AsyncSpinner>> spinners;
    std::vector<std::unique_ptr<ros::CallbackQueue>> cbQueues;
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
        nh.param<std::string>("rootPath", path, ".");
        alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(), std::make_unique<alica::PlanCreator>());

        for (int i = 0; i < getAgentCount(); ++i) {
            cbQueues.emplace_back(std::make_unique<ros::CallbackQueue>());
            spinners.emplace_back(std::make_unique<ros::AsyncSpinner>(4, cbQueues.back().get()));
            alica::AlicaContext* ac =
                    new alica::AlicaContext(AlicaContextParams(getHostName(i), path + "/etc", getRoleSetName(), getMasterPlanName(), stepEngine()));
            alica::AlicaContext* ac =
                    new alica::AlicaContext(AlicaContextParams(getHostName(i), path + "/etc", getRoleSetName(), getMasterPlanName(), stepEngine()));
            ASSERT_TRUE(ac->isValid());
            ac->setCommunicator<alicaRosProxy::AlicaRosCommunication>(*cbQueues.back());
            ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>(*cbQueues.back());
            alica::AlicaEngine* ae = AlicaTestsEngineGetter::getEngine(ac);
            const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
            spinners.back()->start();
            EXPECT_TRUE(ae->init(creators));
            acs.push_back(ac);
            aes.push_back(ae);
        }
    }

    void TearDown() override
    {
        for (auto& spinner : spinners) {
            spinner->stop();
        }
        for (alica::AlicaContext* ac : acs) {
            ac->terminate();
            delete ac;
        }
    }
};
} // namespace supplementary
extern std::jmp_buf restore_point;
void signalHandler(int signal);
