#pragma once

#include <alica_tests/BehaviourCreator.h>
#include <alica_tests/ConditionCreator.h>
#include <alica_tests/ConstraintCreator.h>
#include <alica_tests/ConstraintTestPlanDummySolver.h>
#include <alica_tests/PlanCreator.h>
#include <alica_tests/TestWorldModel.h>
#include <alica_tests/TransitionConditionCreator.h>
#include <alica_tests/UtilityFunctionCreator.h>
#include <alica_tests/test_sched_world_model.h>

#include <alica_tests/TestTracing.h>
#include <alica_timer.h>
#include <communication/AlicaDummyCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaContext.h>
#include <engine/AlicaEngine.h>
#include <engine/logging/AlicaDefaultLogger.h>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <csetjmp>
#include <csignal>
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
    alica::AlicaContext* ac{nullptr};
    alica::AlicaEngine* ae{nullptr};
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
        std::string path = "";
#if defined(PLANS)
        path = PLANS;
        path += "/src/test";
#endif

        ac = new alica::AlicaContext(alica::AlicaContextParams(getHostName(), path + "/etc/", getRoleSetName(), getMasterPlanName(), stepEngine()));

        ASSERT_TRUE(ac->isValid());
        const YAML::Node& config = ac->getConfig();
        ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
        ac->setWorldModel<alicaTests::TestWorldModel>();
        ac->setTimerFactory<alicaTimer::AlicaTestTimerFactory>();
        ac->setLogger<alica::AlicaDefaultLogger>();
        creators = {std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
                std::make_unique<alica::TransitionConditionCreator>()};
        EXPECT_EQ(0, ac->init(std::move(creators), getDelayStart()));
        ae = AlicaTestsEngineGetter::getEngine(ac);
    }

    virtual void TearDown() override
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

class AlicaTestMultiAgentFixtureBase : public ::testing::Test
{
protected:
    std::vector<alica::AlicaContext*> acs;
    std::vector<alica::AlicaEngine*> aes;
};

class TestClock : public AlicaClock
{
public:
    TestClock()
            : _now(AlicaClock::now())
    {
    }
    AlicaTime now() const override { return _now; }
    void increment(AlicaTime t) { _now += t; }

private:
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
        std::string path = "";
#if defined(PLANS)
        path = PLANS;
        path += "/src/test";
#endif
        for (int i = 0; i < getAgentCount(); ++i) {
            creators = {std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                    std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
                    std::make_unique<alica::TransitionConditionCreator>()};

            alica::AlicaContext* ac =
                    new alica::AlicaContext(alica::AlicaContextParams(getHostName(i), path + "/etc/", getRoleSetName(), getMasterPlanName(), stepEngine()));
            ASSERT_TRUE(ac->isValid());
            ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
            ac->setWorldModel<alicaTests::TestWorldModel>();
            ac->setTimerFactory<alicaTimer::AlicaTestTimerFactory>();
            ac->setLogger<alica::AlicaDefaultLogger>();
            if (getUseTestClock()) {
                ac->setClock<TestClock>();
            }
            ac->init(std::move(creators), getDelayStart());
            alica::AlicaEngine* ae = AlicaTestsEngineGetter::getEngine(ac);
            const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
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
        std::string path = "";
#if defined(PLANS)
        path = PLANS;
        path += "/src/test";
#endif
        ac = new alica::AlicaContext(alica::AlicaContextParams("nase", path + "/etc/", getRoleSetName(), getMasterPlanName(), stepEngine()));
        ASSERT_TRUE(ac->isValid());
        const YAML::Node& config = ac->getConfig();
        ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
        ac->setWorldModel<alicaTests::TestWorldModel>();
        ac->setTimerFactory<alicaTimer::AlicaTestTimerFactory>();
        ac->setLogger<alica::AlicaDefaultLogger>();
    }

    void TearDown() override { delete ac; }
};

class AlicaSchedulingTestFixture : public AlicaTestFixtureBase
{
private:
    alica::AlicaCreators creators;

protected:
    virtual const char* getRoleSetName() const { return "Roleset"; }
    virtual const char* getMasterPlanName() const = 0;
    virtual bool stepEngine() const { return true; }
    virtual void SetUp() override
    {
        // Path to test configs set by CMake
        std::string path = "";
#if defined(PLANS)
        path = PLANS;
        path += "/src/test";
#endif
        ac = new alica::AlicaContext(alica::AlicaContextParams("nase", path + "/etc/", getRoleSetName(), getMasterPlanName(), stepEngine()));

        ASSERT_TRUE(ac->isValid());
        const YAML::Node& config = ac->getConfig();
        ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
        ac->setWorldModel<alica_test::SchedWM>();
        ac->setTimerFactory<alicaTimer::AlicaTestTimerFactory>();
        ac->setLogger<alica::AlicaDefaultLogger>();
        creators = {std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
                std::make_unique<alica::TransitionConditionCreator>()};

        ac->init(std::move(creators), true);
        ae = AlicaTestsEngineGetter::getEngine(ac);
        const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
    }

    virtual void TearDown() override
    {
        ac->terminate();
        delete ac;
    }
};

class AlicaTestTracingFixture : public AlicaTestFixtureBase
{
protected:
    alica::AlicaCreators creators;

protected:
    virtual const char* getRoleSetName() const { return "Roleset"; }
    virtual const char* getMasterPlanName() const = 0;
    virtual bool stepEngine() const { return true; }
    virtual void manageWorldModel(alica::AlicaContext* ac) { ac->setWorldModel<alica_test::SchedWM>(); }

    virtual void SetUp() override
    {

        // Path to test configs set by CMake
        std::string path = "";
#if defined(PLANS)
        path = PLANS;
        path += "/src/test";
#endif
        ac = new alica::AlicaContext(alica::AlicaContextParams("nase", path + "/etc/", getRoleSetName(), getMasterPlanName(), stepEngine()));

        ASSERT_TRUE(ac->isValid());
        const YAML::Node& config = ac->getConfig();
        ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
        ac->setTraceFactory<alicaTestTracing::AlicaTestTraceFactory>();
        manageWorldModel(ac);
        ac->setTimerFactory<alicaTimer::AlicaTestTimerFactory>();
        ac->setLogger<alica::AlicaDefaultLogger>();
        creators = {std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
                std::make_unique<alica::TransitionConditionCreator>()};
        ac->init(std::move(creators), true);
        ae = AlicaTestsEngineGetter::getEngine(ac);
        const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
    }

    virtual void TearDown() override
    {
        ac->terminate();
        delete ac;
    }
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
    virtual void manageWorldModel(alica::AlicaContext* ac) { ac->setWorldModel<alica_test::SchedWM>(); }
    virtual alica::AlicaTime getDiscoveryTimeout() const { return alica::AlicaTime::milliseconds(100); }

    void SetUp() override
    {
        // Path to test configs set by CMake
        std::string path = "";
#if defined(PLANS)
        path = PLANS;
        path += "/src/test";
#endif
        for (int i = 0; i < getAgentCount(); ++i) {
            creators = {std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                    std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
                    std::make_unique<alica::TransitionConditionCreator>()};

            alica::AlicaContext* ac =
                    new alica::AlicaContext(alica::AlicaContextParams(getHostName(i), path + "/etc/", getRoleSetName(), getMasterPlanName(), stepEngine()));
            ASSERT_TRUE(ac->isValid());
            ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
            ac->setTraceFactory<alicaTestTracing::AlicaTestTraceFactory>();
            manageWorldModel(ac);
            auto tf = ac->getTraceFactory();
            auto attf = dynamic_cast<alicaTestTracing::AlicaTestTraceFactory*>(tf);
            attf->setWorldModel(ac->getWorldModel());

            ac->setTimerFactory<alicaTimer::AlicaTestTimerFactory>();
            ac->setLogger<alica::AlicaDefaultLogger>();
            ac->init(std::move(creators), true);
            alica::AlicaEngine* ae = AlicaTestsEngineGetter::getEngine(ac);
            const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
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
} // namespace alica

extern std::jmp_buf restore_point;
void signalHandler(int signal);
