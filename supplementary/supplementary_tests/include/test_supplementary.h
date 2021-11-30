#pragma once

#include "PlanCreator.h"
#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "ConstraintTestPlanDummySolver.h"
#include "UtilityFunctionCreator.h"
#include "SupplementaryWorldModel.h"

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
        ac = new alica::AlicaContext(AlicaContextParams("nase",path + "/etc", getRoleSetName(), getMasterPlanName(), stepEngine()));
        ASSERT_TRUE(ac->isValid());
        ac->setCommunicator<alicaRosProxy::AlicaRosCommunication>();
        ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>(4);
        ac->setWorldModel<SupplementaryWorldModel>();
        alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                                      std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(),
                                      std::make_unique<alica::PlanCreator>());
        ae = AlicaTestsEngineGetter::getEngine(ac);
        const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
        EXPECT_TRUE(ae->init(creators));
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
        ac->addSolver<alica::reasoner::CGSolver>();
    }
    void TearDown() override { AlicaTestFixture::TearDown(); }
};

class AlicaTestMultiAgentFixtureBase : public ::testing::Test
{
protected:
    std::vector<alica::AlicaContext*> acs;
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
        nh.param<std::string>("rootPath", path, ".");
        alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(),
                std::make_unique<alica::PlanCreator>());

        for (int i = 0; i < getAgentCount(); ++i) {
            alica::AlicaContext* ac = new alica::AlicaContext(AlicaContextParams(getHostName(i),path+ "/etc", getRoleSetName(), getMasterPlanName(), stepEngine()));
            ASSERT_TRUE(ac->isValid());
            ac->setCommunicator<alicaRosProxy::AlicaRosCommunication>();
            ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>(4);
            ac->setWorldModel<SupplementaryWorldModel>();
            alica::AlicaEngine* ae = AlicaTestsEngineGetter::getEngine(ac);
            const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
            EXPECT_TRUE(ae->init(creators));
            acs.push_back(ac);
            aes.push_back(ae);
        }
    }

    void TearDown() override
    {
        for (alica::AlicaContext* ac : acs) {
            ac->terminate();
            delete ac;
        }
    }
};
} // namespace supplementary
extern std::jmp_buf restore_point;
void signalHandler(int signal);
