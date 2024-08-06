#pragma once

#include <alica/test/TestContext.h>
#include <alica/test/Util.h>
#include <alica_tests/ConstraintTestPlanDummySolver.h>
#include <alica_tests/TestWorldModel.h>
#include <alica_tests/test_sched_world_model.h>

#include <alica_tests/TestTracing.h>
#include <communication/AlicaDummyCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaContext.h>
#include <engine/AlicaEngine.h>
#include <engine/AlicaTimer.h>
#include <engine/logging/AlicaDefaultLogger.h>

#include <DynamicBehaviourCreator.h>
#include <DynamicConditionCreator.h>
#include <DynamicConstraintCreator.h>
#include <DynamicPlanCreator.h>
#include <DynamicTransitionConditionCreator.h>
#include <DynamicUtilityFunctionCreator.h>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <csetjmp>
#include <csignal>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>

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

class AlicaTestBase
{
public:
    AlicaTestBase()
#if defined(THIS_PACKAGE_DIR)
            : _testFolderPath(std::string(THIS_PACKAGE_DIR) + "/etc/")
#endif
    {
    }

    virtual std::vector<std::string> getTestFolderPaths() const { return {_testFolderPath}; }

private:
    std::string _testFolderPath;
};

class AlicaTestFixtureBase : public ::testing::Test, public AlicaTestBase
{
protected:
    AlicaTestFixtureBase() = default;
    virtual const char* getHostName() const { return "nase"; }
    virtual const char* getRoleSetName() const { return "Roleset"; }

    std::unique_ptr<alica::AlicaContext> ac;
    alica::AlicaEngine* ae{nullptr};
};

class AlicaTestFixture : public AlicaTestFixtureBase
{
private:
    alica::AlicaCreators creators;

protected:
    AlicaTestFixture() = default;
    virtual bool getDelayStart() { return false; }
    virtual const char* getRoleSetName() const { return "Roleset"; }
    virtual const char* getMasterPlanName() const = 0;
    virtual bool stepEngine() const { return true; }
    virtual void SetUp() override
    {
        ac = std::make_unique<alica::AlicaContext>(
                alica::AlicaContextParams(getHostName(), getTestFolderPaths(), getRoleSetName(), getMasterPlanName(), stepEngine()));

        ASSERT_TRUE(ac->isValid());
        const YAML::Node& config = ac->getConfig();
        ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
        ac->setTimerFactory<alica::AlicaSystemTimerFactory>();
        ac->setLogger<alica::AlicaDefaultLogger>();
        creators = {std::make_unique<alica::DynamicConditionCreator>(), std::make_unique<alica::DynamicUtilityFunctionCreator>(),
                std::make_unique<alica::DynamicConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(),
                std::make_unique<alica::DynamicPlanCreator>(), std::make_unique<alica::DynamicTransitionConditionCreator>()};
        EXPECT_EQ(0, ac->init(std::move(creators), getDelayStart()));

        ae = AlicaTestsEngineGetter::getEngine(ac.get());
        LockedBlackboardRW(ac->editGlobalBlackboard()).set("worldmodel", std::make_shared<alicaTests::TestWorldModel>());
    }

    virtual void TearDown() override {}
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

class AlicaTestMultiAgentFixtureBase : public ::testing::Test, public AlicaTestBase
{
protected:
    AlicaTestMultiAgentFixtureBase(){};
    virtual const char* getHostName(int agentNumber) const = 0;
    virtual const char* getRoleSetName() const = 0;

    std::vector<std::unique_ptr<alica::AlicaContext>> acs;
    std::vector<alica::AlicaEngine*> aes;
};

class TestClock : public AlicaClock
{
public:
    TestClock()
            : _now(AlicaClock::now())
    {
    }
    AlicaTime now() const override
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _now;
    }
    void increment(AlicaTime t)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _now += t;
    }

private:
    mutable std::mutex _mutex;
    AlicaTime _now;
};

class AlicaTestMultiAgentFixture : public AlicaTestMultiAgentFixtureBase
{
private:
    alica::AlicaCreators creators;

protected:
    AlicaTestMultiAgentFixture() {}
    virtual bool getDelayStart() { return true; }
    virtual bool getUseTestClock() { return false; }

    virtual const char* getMasterPlanName() const = 0;
    virtual int getAgentCount() const = 0;
    virtual bool stepEngine() const { return true; }
    virtual alica::AlicaTime getDiscoveryTimeout() const { return alica::AlicaTime::milliseconds(100); }

    void SetUp() override
    {
        for (int i = 0; i < getAgentCount(); ++i) {
            creators = {std::make_unique<alica::DynamicConditionCreator>(), std::make_unique<alica::DynamicUtilityFunctionCreator>(),
                    std::make_unique<alica::DynamicConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(),
                    std::make_unique<alica::DynamicPlanCreator>(), std::make_unique<alica::DynamicTransitionConditionCreator>()};

            auto ac = std::make_unique<alica::AlicaContext>(
                    alica::AlicaContextParams(getHostName(i), getTestFolderPaths(), getRoleSetName(), getMasterPlanName(), stepEngine()));

            ASSERT_TRUE(ac->isValid());
            ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
            ac->setTimerFactory<alica::AlicaSystemTimerFactory>();
            ac->setLogger<alica::AlicaDefaultLogger>();
            if (getUseTestClock()) {
                ac->setClock<TestClock>();
            }
            ac->init(std::move(creators), getDelayStart());

            auto ae = AlicaTestsEngineGetter::getEngine(ac.get());

            LockedBlackboardRW(ac->editGlobalBlackboard()).set("worldmodel", std::make_shared<alicaTests::TestWorldModel>());
            const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();

            acs.push_back(std::move(ac));
            aes.push_back(ae);
        }
    }
};

class AlicaTestNotInitializedFixture : public AlicaTestFixtureBase
{
private:
    alica::AlicaCreators creators;

protected:
    virtual const char* getMasterPlanName() const = 0;
    virtual bool stepEngine() const { return true; }
    void SetUp() override
    {
        ac = std::make_unique<alica::AlicaContext>(
                alica::AlicaContextParams(getHostName(), getTestFolderPaths(), getRoleSetName(), getMasterPlanName(), stepEngine()));

        ASSERT_TRUE(ac->isValid());
        const YAML::Node& config = ac->getConfig();
        ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
        ac->setTimerFactory<alica::AlicaSystemTimerFactory>();
        ac->setLogger<alica::AlicaDefaultLogger>();
    }
};

class AlicaSchedulingTestFixture : public AlicaTestFixtureBase
{
private:
    alica::AlicaCreators creators;

protected:
    virtual const char* getMasterPlanName() const = 0;
    virtual bool stepEngine() const { return true; }
    virtual void manageWorldModel(alica::AlicaContext* ac)
    {
        LockedBlackboardRW(ac->editGlobalBlackboard()).set("worldmodel", std::make_shared<alica_test::SchedWM>());
    }
    virtual void SetUp() override
    {
        ac = std::make_unique<alica::AlicaContext>(
                alica::AlicaContextParams(getHostName(), getTestFolderPaths(), getRoleSetName(), getMasterPlanName(), stepEngine()));

        ASSERT_TRUE(ac->isValid());
        const YAML::Node& config = ac->getConfig();
        ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
        ac->setTimerFactory<alica::AlicaSystemTimerFactory>();
        ac->setLogger<alica::AlicaDefaultLogger>();
        creators = {std::make_unique<alica::DynamicConditionCreator>(), std::make_unique<alica::DynamicUtilityFunctionCreator>(),
                std::make_unique<alica::DynamicConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(),
                std::make_unique<alica::DynamicPlanCreator>(), std::make_unique<alica::DynamicTransitionConditionCreator>()};

        ac->init(std::move(creators), true);
        manageWorldModel(ac.get());

        ae = AlicaTestsEngineGetter::getEngine(ac.get());
        const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
    }
};

class AlicaTestTracingFixture : public AlicaTestFixtureBase
{
protected:
    alica::AlicaCreators creators;

protected:
    virtual const char* getMasterPlanName() const = 0;
    virtual bool stepEngine() const { return true; }
    virtual void manageWorldModel(alica::AlicaContext* ac)
    {
        LockedBlackboardRW(ac->editGlobalBlackboard()).set("worldmodel", std::make_shared<alica_test::SchedWM>());
    }

    virtual void SetUp() override
    {
        ac = std::make_unique<alica::AlicaContext>(
                alica::AlicaContextParams(getHostName(), getTestFolderPaths(), getRoleSetName(), getMasterPlanName(), stepEngine()));

        ASSERT_TRUE(ac->isValid());
        const YAML::Node& config = ac->getConfig();
        ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
        ac->setTraceFactory<alicaTestTracing::AlicaTestTraceFactory>();
        ac->setTimerFactory<alica::AlicaSystemTimerFactory>();
        ac->setLogger<alica::AlicaDefaultLogger>();
        creators = {std::make_unique<alica::DynamicConditionCreator>(), std::make_unique<alica::DynamicUtilityFunctionCreator>(),
                std::make_unique<alica::DynamicConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(),
                std::make_unique<alica::DynamicPlanCreator>(), std::make_unique<alica::DynamicTransitionConditionCreator>()};
        ac->init(std::move(creators), true);
        manageWorldModel(ac.get());

        ae = AlicaTestsEngineGetter::getEngine(ac.get());
        const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
    }
};

class AlicaTestMultiAgentTracingFixture : public AlicaTestMultiAgentFixtureBase
{
protected:
    alica::AlicaCreators creators;

protected:
    AlicaTestMultiAgentTracingFixture(){};

    virtual const char* getMasterPlanName() const = 0;
    virtual int getAgentCount() const = 0;
    virtual bool stepEngine() const { return true; }
    virtual void manageWorldModel(alica::AlicaContext* ac)
    {
        LockedBlackboardRW(ac->editGlobalBlackboard()).set("worldmodel", std::make_shared<alica_test::SchedWM>());
    }
    virtual alica::AlicaTime getDiscoveryTimeout() const { return alica::AlicaTime::milliseconds(100); }

    void SetUp() override
    {
        for (int i = 0; i < getAgentCount(); ++i) {
            creators = {std::make_unique<alica::DynamicConditionCreator>(), std::make_unique<alica::DynamicUtilityFunctionCreator>(),
                    std::make_unique<alica::DynamicConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(),
                    std::make_unique<alica::DynamicPlanCreator>(), std::make_unique<alica::DynamicTransitionConditionCreator>()};

            auto ac = std::make_unique<alica::AlicaContext>(
                    alica::AlicaContextParams(getHostName(i), getTestFolderPaths(), getRoleSetName(), getMasterPlanName(), stepEngine()));

            ASSERT_TRUE(ac->isValid());
            ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
            ac->setTraceFactory<alicaTestTracing::AlicaTestTraceFactory>();

            ac->setTimerFactory<alica::AlicaSystemTimerFactory>();
            ac->setLogger<alica::AlicaDefaultLogger>();
            ac->init(std::move(creators), true);
            manageWorldModel(ac.get());

            auto ae = AlicaTestsEngineGetter::getEngine(ac.get());
            auto tf = ac->getTraceFactory();
            auto attf = dynamic_cast<alicaTestTracing::AlicaTestTraceFactory*>(tf);
            attf->setWorldModel(const_cast<alica::Blackboard*>(&ac->getGlobalBlackboard()));
            const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();

            acs.push_back(std::move(ac));
            aes.push_back(ae);
        }
    }
};
} // namespace alica

namespace alica::test
{
class SingleAgentUninitializedTestFixture : public ::testing::Test
{
public:
    virtual std::string getPlaceholderMappingFileName() const { return "placeholder_mapping.json"; }
    virtual void SetUp() override
    {
        // Path to test configs set by CMake
        std::string path;
#if defined(THIS_PACKAGE_DIR)
        path = THIS_PACKAGE_DIR;
#endif
        std::cerr << "create with " << path + "/config/" + getPlaceholderMappingFileName() << std::endl;
        _tc = std::make_unique<TestContext>(agentName(), std::vector<std::string>{path + "/etc/"}, "Roleset", getMasterPlanName(), true, 1,
                readPlaceholderMapping(path + "/config/" + getPlaceholderMappingFileName()));
        ASSERT_TRUE(_tc->isValid());
    }

    void initialize()
    {
        _tc->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
        _tc->setTimerFactory<alica::AlicaSystemTimerFactory>();

        AlicaCreators creators{std::make_unique<alica::DynamicConditionCreator>(), std::make_unique<alica::DynamicUtilityFunctionCreator>(),
                std::make_unique<alica::DynamicConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(),
                std::make_unique<alica::DynamicPlanCreator>(), std::make_unique<alica::DynamicTransitionConditionCreator>()};
        _tc->init(std::move(creators));
        _tc->startEngine();

        STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan(getMasterPlanName())) << _tc->getLastFailure();
    }

    virtual void TearDown() override {}

    std::string agentName() const { return "hairy"; }

protected:
    std::optional<std::string> readPlaceholderMapping(const std::string& placeholderMappingFilePath)
    {
        std::optional<std::string> maybeMapping;
        if (!placeholderMappingFilePath.empty()) {
            std::ifstream mappingFileStream(placeholderMappingFilePath);
            if (mappingFileStream) {
                std::ostringstream ss;
                ss << mappingFileStream.rdbuf();
                maybeMapping.emplace(ss.str());
            } else {
                std::cerr << "Could not read mapping file" << std::endl;
            }
        }
        return maybeMapping;
    }

    virtual const char* getMasterPlanName() const { return "TestMasterPlan"; }

    std::unique_ptr<TestContext> _tc;
};

class SingleAgentTestFixture : public SingleAgentUninitializedTestFixture
{
    using Base = SingleAgentUninitializedTestFixture;

public:
    virtual void SetUp() override
    {
        Base::SetUp();
        Base::initialize();
    }

    virtual void TearDown() override { Base::TearDown(); }

protected:
    virtual const char* getMasterPlanName() const override { return "TestMasterPlan"; }
};

} // namespace alica::test

extern std::jmp_buf restore_point;
void signalHandler(int signal);
