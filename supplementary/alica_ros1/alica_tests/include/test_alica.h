#pragma once

#include <alica_tests/ConditionCreator.h>
#include <alica_tests/ConstraintCreator.h>
#include <alica_tests/ConstraintTestPlanDummySolver.h>
#include <alica_tests/LegacyTransitionConditionCreator.h>
#include <alica_tests/PlanCreator.h>
#include <alica_tests/TestWorldModel.h>
#include <alica_tests/TransitionConditionCreator.h>
#include <alica_tests/UtilityFunctionCreator.h>
#include <alica_tests/test_sched_world_model.h>

#include <alica_tests/TestTracing.h>
#include <communication/AlicaDummyCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaContext.h>
#include <engine/AlicaEngine.h>
#include <engine/AlicaTimer.h>
#include <engine/logging/AlicaDefaultLogger.h>

#include <DynamicBehaviourCreator.h>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <csetjmp>
#include <csignal>
#include <mutex>
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

class AlicaTestFixtureBase : public ::testing::Test
{
protected:
    std::unique_ptr<alica::AlicaContext> ac;
    std::unique_ptr<alica::AlicaEngine> ae;
};

class AlicaTestFixture : public AlicaTestFixtureBase
{
private:
    alica::AlicaCreators creators;

protected:
    virtual bool getDelayStart() { return false; }
    virtual const char* getRoleSetName() const { return "Roleset"; }
    virtual const char* getMasterPlanName() const = 0;
    virtual bool stepEngine() const { return true; }
    virtual const char* getHostName() const { return "nase"; }
    virtual void SetUp() override
    {
        // Path to test configs set by CMake
        std::string path;
#if defined(PLANS)
        path = PLANS;
        path += "/src/test";
#endif

        ac = std::make_unique<alica::AlicaContext>(alica::AlicaContextParams("nase", path + "/etc/", getRoleSetName(), getMasterPlanName(), stepEngine()));

        ASSERT_TRUE(ac->isValid());
        const YAML::Node& config = ac->getConfig();
        ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
        ac->setTimerFactory<alicaTimer::AlicaTestTimerFactory>();
        ac->setLogger<alica::AlicaDefaultLogger>();
        creators = {std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
                std::make_unique<alica::TransitionConditionCreator>()};
        EXPECT_EQ(0, ac->init(std::move(creators), getDelayStart()));
        std::unique_ptr<alica::AlicaEngine> ae(AlicaTestsEngineGetter::getEngine(ac.get()));
        LockedBlackboardRW(ac->editGlobalBlackboard()).set("worldmodel", std::make_shared<alicaTests::TestWorldModel>());
    }

    virtual void TearDown() override { ac->terminate(); }
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

class AlicaTestMultiAgentFixtureBase : public ::testing::Test
{
protected:
    std::vector<std::unique_ptr<alica::AlicaContext>> acs;
    std::vector<std::unique_ptr<alica::AlicaEngine>> aes;
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
    virtual bool getDelayStart() { return true; }
    virtual bool getUseTestClock() { return false; }
    virtual const char* getRoleSetName() const { return "Roleset"; }
    virtual const char* getMasterPlanName() const = 0;
    virtual int getAgentCount() const = 0;
    virtual bool stepEngine() const { return true; }
    virtual const char* getHostName(int agentNumber) const { return "nase"; }
    virtual alica::AlicaTime getDiscoveryTimeout() const { return alica::AlicaTime::milliseconds(100); }

    void SetUp() override
    {
        // Path to test configs set by CMake
        std::string path;
#if defined(PLANS)
        path = PLANS;
        path += "/src/test";
#endif
        for (int i = 0; i < getAgentCount(); ++i) {
            creators = {std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                    std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
                    std::make_unique<alica::TransitionConditionCreator>()};

            auto ac = std::make_unique<alica::AlicaContext>(
                    alica::AlicaContextParams("nase", path + "/etc/", getRoleSetName(), getMasterPlanName(), stepEngine()));

            ASSERT_TRUE(ac->isValid());
            ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
            ac->setTimerFactory<alicaTimer::AlicaTestTimerFactory>();
            ac->setLogger<alica::AlicaDefaultLogger>();
            if (getUseTestClock()) {
                ac->setClock<TestClock>();
            }
            ac->init(std::move(creators), getDelayStart());

            std::unique_ptr<alica::AlicaEngine> ae(AlicaTestsEngineGetter::getEngine(ac.get()));

            LockedBlackboardRW(ac->editGlobalBlackboard()).set("worldmodel", std::make_shared<alicaTests::TestWorldModel>());
            const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();

            acs.push_back(std::move(ac));
            aes.push_back(std::move(ae));
        }
    }

    void TearDown() override
    {
        for (auto& ac : acs) {
            ac->terminate();
        }
    }
};

class AlicaTestNotInitializedFixture : public AlicaTestFixtureBase
{
private:
    alica::AlicaCreators creators;

protected:
    virtual const char* getRoleSetName() const { return "Roleset"; }
    virtual const char* getMasterPlanName() const = 0;
    virtual bool stepEngine() const { return true; }
    void SetUp() override
    {
        // Path to test configs set by CMake
        std::string path;
#if defined(PLANS)
        path = PLANS;
        path += "/src/test";
#endif
        ac = std::make_unique<alica::AlicaContext>(alica::AlicaContextParams("nase", path + "/etc/", getRoleSetName(), getMasterPlanName(), stepEngine()));

        ASSERT_TRUE(ac->isValid());
        const YAML::Node& config = ac->getConfig();
        ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
        ac->setTimerFactory<alicaTimer::AlicaTestTimerFactory>();
        ac->setLogger<alica::AlicaDefaultLogger>();
    }

    void TearDown() override { ac->terminate(); }
};

class AlicaSchedulingTestFixture : public AlicaTestFixtureBase
{
private:
    alica::AlicaCreators creators;

protected:
    virtual const char* getRoleSetName() const { return "Roleset"; }
    virtual const char* getMasterPlanName() const = 0;
    virtual bool stepEngine() const { return true; }
    virtual void manageWorldModel(alica::AlicaContext* ac)
    {
        LockedBlackboardRW(ac->editGlobalBlackboard()).set("worldmodel", std::make_shared<alica_test::SchedWM>());
    }
    virtual void SetUp() override
    {
        // Path to test configs set by CMake
        std::string path;
#if defined(PLANS)
        path = PLANS;
        path += "/src/test";
#endif
        ac = std::make_unique<alica::AlicaContext>(alica::AlicaContextParams("nase", path + "/etc/", getRoleSetName(), getMasterPlanName(), stepEngine()));

        ASSERT_TRUE(ac->isValid());
        const YAML::Node& config = ac->getConfig();
        ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
        ac->setTimerFactory<alicaTimer::AlicaTestTimerFactory>();
        ac->setLogger<alica::AlicaDefaultLogger>();
        creators = {std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
                std::make_unique<alica::TransitionConditionCreator>()};
        ac->init(std::move(creators), true);
        manageWorldModel(ac.get());

        std::unique_ptr<alica::AlicaEngine> ae(AlicaTestsEngineGetter::getEngine(ac.get()));
        const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
    }

    virtual void TearDown() override { ac->terminate(); }
};

class AlicaTestTracingFixture : public AlicaTestFixtureBase
{
protected:
    alica::AlicaCreators creators;

protected:
    virtual const char* getRoleSetName() const { return "Roleset"; }
    virtual const char* getMasterPlanName() const = 0;
    virtual bool stepEngine() const { return true; }
    virtual void manageWorldModel(alica::AlicaContext* ac)
    {
        LockedBlackboardRW(ac->editGlobalBlackboard()).set("worldmodel", std::make_shared<alica_test::SchedWM>());
    }

    virtual void SetUp() override
    {

        // Path to test configs set by CMake
        std::string path;
#if defined(PLANS)
        path = PLANS;
        path += "/src/test";
#endif
        ac = std::make_unique<alica::AlicaContext>(alica::AlicaContextParams("nase", path + "/etc/", getRoleSetName(), getMasterPlanName(), stepEngine()));

        ASSERT_TRUE(ac->isValid());
        const YAML::Node& config = ac->getConfig();
        ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
        ac->setTraceFactory<alicaTestTracing::AlicaTestTraceFactory>();
        ac->setTimerFactory<alicaTimer::AlicaTestTimerFactory>();
        ac->setLogger<alica::AlicaDefaultLogger>();
        creators = {std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
                std::make_unique<alica::TransitionConditionCreator>()};
        ac->init(std::move(creators), true);
        manageWorldModel(ac.get());

        std::unique_ptr<alica::AlicaEngine> ae(AlicaTestsEngineGetter::getEngine(ac.get()));
        const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
    }

    virtual void TearDown() override { ac->terminate(); }
};

class AlicaLegacyConditionsFixture : public AlicaTestFixtureBase
{
protected:
    alica::AlicaCreators creators;

protected:
    virtual const char* getRoleSetName() const { return "Roleset"; }
    virtual const char* getMasterPlanName() const = 0;
    virtual bool stepEngine() const { return true; }
    virtual void manageWorldModel(alica::AlicaContext* ac)
    {
        LockedBlackboardRW(ac->editGlobalBlackboard()).set("worldmodel", std::make_shared<alica_test::SchedWM>());
    }

    // same setup as AlicaSchedulingTestFixture, but use LegacyTransitionConditionCreator instead of TransitionConditionCreator
    virtual void SetUp() override
    {
        // Path to test configs set by CMake
        std::string path;
#if defined(PLANS)
        path = PLANS;
        path += "/src/test";
#endif
        auto ac = std::make_unique<alica::AlicaContext>(alica::AlicaContextParams("nase", path + "/etc/", getRoleSetName(), getMasterPlanName(), stepEngine()));

        ASSERT_TRUE(ac->isValid());
        const YAML::Node& config = ac->getConfig();
        ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
        ac->setTimerFactory<alicaTimer::AlicaTestTimerFactory>();
        ac->setLogger<alica::AlicaDefaultLogger>();
        creators = {std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
                std::make_unique<alica::LegacyTransitionConditionCreator>()};

        ac->init(std::move(creators), true);
        manageWorldModel(ac.get());

        std::unique_ptr<alica::AlicaEngine> ae(AlicaTestsEngineGetter::getEngine(ac.get()));
        const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
    }

    void TearDown() override { ac->terminate(); }
};

class AlicaTestMultiAgentTracingFixture : public AlicaTestMultiAgentFixtureBase
{
protected:
    alica::AlicaCreators creators;

protected:
    virtual const char* getRoleSetName() const { return "Roleset"; }
    virtual const char* getMasterPlanName() const = 0;
    virtual int getAgentCount() const = 0;
    virtual bool stepEngine() const { return true; }
    virtual const char* getHostName(int agentNumber) const { return "nase"; }
    virtual void manageWorldModel(alica::AlicaContext* ac)
    {
        LockedBlackboardRW(ac->editGlobalBlackboard()).set("worldmodel", std::make_shared<alica_test::SchedWM>());
    }
    virtual alica::AlicaTime getDiscoveryTimeout() const { return alica::AlicaTime::milliseconds(100); }

    void SetUp() override
    {
        // Path to test configs set by CMake
        std::string path;
#if defined(PLANS)
        path = PLANS;
        path += "/src/test";
#endif
        for (int i = 0; i < getAgentCount(); ++i) {
            creators = {std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                    std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
                    std::make_unique<alica::TransitionConditionCreator>()};

            auto ac = std::make_unique<alica::AlicaContext>(
                    alica::AlicaContextParams("nase", path + "/etc/", getRoleSetName(), getMasterPlanName(), stepEngine()));

            ASSERT_TRUE(ac->isValid());
            ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
            ac->setTraceFactory<alicaTestTracing::AlicaTestTraceFactory>();

            ac->setTimerFactory<alicaTimer::AlicaTestTimerFactory>();
            ac->setLogger<alica::AlicaDefaultLogger>();
            ac->init(std::move(creators), true);
            manageWorldModel(ac.get());

            std::unique_ptr<alica::AlicaEngine> ae(AlicaTestsEngineGetter::getEngine(ac.get()));
            auto tf = ac->getTraceFactory();
            auto attf = dynamic_cast<alicaTestTracing::AlicaTestTraceFactory*>(tf);
            attf->setWorldModel(const_cast<alica::Blackboard*>(&ac->getGlobalBlackboard()));
            const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();

            acs.push_back(std::move(ac));
            aes.push_back(std::move(ae));
        }
    }

    void TearDown() override
    {
        for (auto& ac : acs) {
            ac->terminate();
        }
    }
};
} // namespace alica

extern std::jmp_buf restore_point;
void signalHandler(int signal);
