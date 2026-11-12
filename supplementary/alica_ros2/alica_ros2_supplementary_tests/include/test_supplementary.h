#pragma once

// The ROS2 fixtures of the supplementary tests. The test sources are those of the ROS1 package supplementary_tests,
// which include this header by the name of the ROS1 header.

#include "ConstraintTestPlanDummySolver.h"
#include <DynamicBehaviourCreator.h>
#include <DynamicConditionCreator.h>
#include <DynamicConstraintCreator.h>
#include <DynamicPlanCreator.h>
#include <DynamicTransitionConditionCreator.h>
#include <DynamicUtilityFunctionCreator.h>
#include <clock/AlicaRosTimer.h>
#include <communication/AlicaRosCommunication.h>
#include <constraintsolver/CGSolver.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaContext.h>
#include <engine/AlicaEngine.h>
#include <engine/AlicaTimer.h>

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <csetjmp>
#include <csignal>
#include <memory>
#include <string>
#include <thread>
#include <vector>

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

// The directory that holds etc/ with the plans. ROS1 reads it from the parameter rootPath, here the build sets it.
inline std::string getTestRootPath()
{
    return SUPPLEMENTARY_TESTS_ROOT_PATH;
}

/**
 * A node with its own executor and four threads. It takes the place of the ROS1 callback queue with its AsyncSpinner:
 * the communication and the timers of one agent run on this node.
 */
class NodeSpinner
{
public:
    explicit NodeSpinner(const std::string& nodeName)
            : _node(rclcpp::Node::make_shared(nodeName))
            , _executor(rclcpp::ExecutorOptions(), 4)
    {
        _executor.add_node(_node);
    }

    ~NodeSpinner() { stop(); }

    rclcpp::Node::SharedPtr node() const { return _node; }

    void start()
    {
        if (!_thread.joinable()) {
            _thread = std::thread([this]() {
                _executor.spin();
                _spinReturned = true;
            });
        }
    }

    void stop()
    {
        if (_thread.joinable()) {
            // cancel() has no effect on an executor that does not spin yet.
            // spin() also returns without cancel() when the context shuts down, for example on SIGINT.
            while (!_executor.is_spinning() && !_spinReturned) {
                std::this_thread::yield();
            }
            _executor.cancel();
            _thread.join();
        }
    }

private:
    rclcpp::Node::SharedPtr _node;
    rclcpp::executors::MultiThreadedExecutor _executor;
    std::thread _thread;
    std::atomic<bool> _spinReturned{false};
};

class AlicaTestFixtureBase : public ::testing::Test
{
protected:
    alica::AlicaContext* ac;
    alica::AlicaEngine* ae;
};

class AlicaTestFixture : public AlicaTestFixtureBase
{
private:
    alica::AlicaCreators creators;

protected:
    virtual const char* getRoleSetName() const { return "Roleset"; }
    virtual const char* getMasterPlanName() const = 0;
    virtual bool stepEngine() const { return true; }
    void SetUp() override
    {
        const std::string path = getTestRootPath();
        ac = new alica::AlicaContext(AlicaContextParams("nase", {path + "/etc"}, getRoleSetName(), getMasterPlanName(), stepEngine()));
        ASSERT_TRUE(ac->isValid());
        spinner = std::make_unique<NodeSpinner>("alica_supplementary_tests");
        ac->setCommunicator<alicaRosProxy::AlicaRosCommunication>(spinner->node());
        ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>(spinner->node());
        creators = {std::make_unique<alica::DynamicConditionCreator>(), std::make_unique<alica::DynamicUtilityFunctionCreator>(),
                std::make_unique<alica::DynamicConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(),
                std::make_unique<alica::DynamicPlanCreator>(), std::make_unique<alica::DynamicTransitionConditionCreator>()};
        ac->init(std::move(creators), true);
        ae = AlicaTestsEngineGetter::getEngine(ac);
        const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
        spinner->start();
    }

    void TearDown() override
    {
        if (spinner) {
            spinner->stop();
        }
        ac->terminate();
        delete ac;
    }

    std::unique_ptr<NodeSpinner> spinner;
};

class AlicaTestFixtureWithSolvers : public AlicaTestFixture
{
protected:
    void SetUp() override
    {
        AlicaTestFixture::SetUp();
        ac->addSolver<alica::reasoner::ConstraintTestPlanDummySolver>();
        ac->addSolver<alica::reasoner::CGSolver>(ac->getConfig());
    }
    void TearDown() override { AlicaTestFixture::TearDown(); }
};

class AlicaTestMultiAgentFixtureBase : public ::testing::Test
{
protected:
    std::vector<alica::AlicaContext*> acs;
    std::vector<alica::AlicaEngine*> aes;
    // One node and executor for each agent, as each agent had its own callback queue in ROS1
    std::vector<std::unique_ptr<NodeSpinner>> spinners;
};

class AlicaTestMultiAgentFixture : public AlicaTestMultiAgentFixtureBase
{
private:
    alica::AlicaCreators creators;

protected:
    virtual const char* getRoleSetName() const { return "Roleset"; }
    virtual const char* getMasterPlanName() const = 0;
    virtual int getAgentCount() const = 0;
    virtual bool stepEngine() const { return true; }
    virtual const char* getHostName(int agentNumber) const { return "nase"; }
    virtual alica::AlicaTime getDiscoveryTimeout() const { return alica::AlicaTime::milliseconds(100); }

    void SetUp() override
    {
        const std::string path = getTestRootPath();

        for (int i = 0; i < getAgentCount(); ++i) {
            creators = {std::make_unique<alica::DynamicConditionCreator>(), std::make_unique<alica::DynamicUtilityFunctionCreator>(),
                    std::make_unique<alica::DynamicConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(),
                    std::make_unique<alica::DynamicPlanCreator>(), std::make_unique<alica::DynamicTransitionConditionCreator>()};

            spinners.emplace_back(std::make_unique<NodeSpinner>("alica_supplementary_tests_agent_" + std::to_string(i)));
            alica::AlicaContext* ac =
                    new alica::AlicaContext(AlicaContextParams(getHostName(i), {path + "/etc"}, getRoleSetName(), getMasterPlanName(), stepEngine()));
            ASSERT_TRUE(ac->isValid());
            ac->setCommunicator<alicaRosProxy::AlicaRosCommunication>(spinners.back()->node());
            ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>(spinners.back()->node());
            ac->setEngineTimerFactory<alica::AlicaSystemTimerFactory>();
            EXPECT_EQ(0, ac->init(std::move(creators), true));
            alica::AlicaEngine* ae = AlicaTestsEngineGetter::getEngine(ac);
            const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
            spinners.back()->start();
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
