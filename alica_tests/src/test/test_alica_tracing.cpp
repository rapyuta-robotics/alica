#include "test_alica.h"

#include <alica_tests/Behaviour/Attack.h>
#include <alica_tests/Behaviour/MidFieldStandard.h>
#include <alica_tests/BehaviourCreator.h>
#include <alica_tests/ConditionCreator.h>
#include <alica_tests/ConstraintCreator.h>
#include <alica_tests/PlanCreator.h>
#include <alica_tests/UtilityFunctionCreator.h>

#include <alica_tests/CounterClass.h>
#include <alica_tests/DummyTestSummand.h>
#include <alica_tests/TestWorldModel.h>

#include <alica/test/Util.h>
#include <communication/AlicaDummyCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/Assignment.h>
#include <engine/BasicBehaviour.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaCommunication.h>
#include <engine/PlanBase.h>
#include <engine/PlanRepository.h>
#include <engine/TeamObserver.h>
#include <engine/UtilityFunction.h>
#include <engine/allocationauthority/AllocationDifference.h>
#include <engine/allocationauthority/EntryPointRobotPair.h>
#include <engine/model/Behaviour.h>
#include <engine/model/Plan.h>
#include <engine/model/RuntimeCondition.h>
#include <engine/model/State.h>
#include <engine/model/Task.h>
#include <engine/modelmanagement/factories/TaskFactory.h>
#include <engine/teammanager/TeamManager.h>

#include <gtest/gtest.h>

#include <iostream>

namespace alica
{
namespace
{

class AlicaTracingTest : public AlicaTestTracingFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "TestTracingMasterPlan"; }
    bool stepEngine() const override { return false; }
    virtual void SetUp() override
    {
        alicaTests::TestWorldModel::getOne()->reset();
        alicaTests::TestWorldModel::getTwo()->reset();

        // determine the path to the test config
        ros::NodeHandle nh;
        std::string path;
        nh.param<std::string>("/rootPath", path, ".");
        ac = new alica::AlicaContext(alica::AlicaContextParams("nase", path + "/etc/", getRoleSetName(), getMasterPlanName(), stepEngine()));

        ASSERT_TRUE(ac->isValid());
        const YAML::Node& config = ac->getConfig();
        spinner = std::make_unique<ros::AsyncSpinner>(config["Alica"]["ThreadPoolSize"].as<int>(4));
        ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
        ac->setWorldModel<alicaTests::TestWorldModel>();
        ac->setTraceFactory<alicaTestTracing::AlicaTestTraceFactory>();

        auto tf = ac->getTraceFactory();
        auto attf = dynamic_cast<alicaTestTracing::AlicaTestTraceFactory*>(tf);
        attf->setWorldModel(ac->getWorldModel());

        ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>();
        ac->setLogger<alicaRosLogger::AlicaRosLogger>(config["Local"]["ID"].as<int>());
        creators = {std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
                std::make_unique<alica::TransitionConditionCreator>()};
        ac->init(std::move(creators), true);
        ae = AlicaTestsEngineGetter::getEngine(ac);
        const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
        spinner->start();
    }
};

class AlicaAuthorityTracingTest : public AlicaTestMultiAgentTracingFixture
{
protected:
    const int agentCount = 2;
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "AuthorityTestMaster"; }
    int getAgentCount() const override { return agentCount; }
    const char* getHostName(int agentNumber) const override
    {
        if (agentNumber) {
            return "hairy";
        } else {
            return "nase";
        }
    }
    void SetUp() override
    {
        alicaTests::TestWorldModel::getOne()->reset();
        alicaTests::TestWorldModel::getTwo()->reset();
        // determine the path to the test config
        ros::NodeHandle nh;
        std::string path;
        nh.param<std::string>("/rootPath", path, ".");
        for (int i = 0; i < getAgentCount(); ++i) {
            creators = {std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                    std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
                    std::make_unique<alica::TransitionConditionCreator>()};

            alica::AlicaContext* ac =
                    new alica::AlicaContext(alica::AlicaContextParams(getHostName(i), path + "/etc/", getRoleSetName(), getMasterPlanName(), stepEngine()));
            ASSERT_TRUE(ac->isValid());
            cbQueues.emplace_back(std::make_unique<ros::CallbackQueue>());
            spinners.emplace_back(std::make_unique<ros::AsyncSpinner>(4, cbQueues.back().get()));
            ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
            ac->setWorldModel<alicaTests::TestWorldModel>();
            ac->setTraceFactory<alicaTestTracing::AlicaTestTraceFactory>();

            auto tf = ac->getTraceFactory();
            auto attf = dynamic_cast<alicaTestTracing::AlicaTestTraceFactory*>(tf);
            attf->setWorldModel(ac->getWorldModel());

            ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>(*cbQueues.back());
            ac->setLogger<alicaRosLogger::AlicaRosLogger>(ac->getConfig()["Local"]["ID"].as<int>());
            ac->init(std::move(creators), true);
            alica::AlicaEngine* ae = AlicaTestsEngineGetter::getEngine(ac);
            const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
            spinners.back()->start();
            acs.push_back(ac);
            aes.push_back(ae);
        }
    }
};

TEST_F(AlicaTracingTest, runTracing)
{
    ASSERT_NO_SIGNAL
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(200));
    auto twm1 = dynamic_cast<alicaTests::TestWorldModel*>(ac->getWorldModel());

    twm1->setPreCondition1840401110297459509(true);
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(200));

    EXPECT_EQ(twm1->tracingParents["EmptyBehaviour"], "TestTracingSubPlan");
    EXPECT_EQ(twm1->tracingParents["TestTracingSubPlan"], "TestTracingMasterPlan");
}

TEST_F(AlicaAuthorityTracingTest, taskAssignmentTracing)
{
    auto twm1 = dynamic_cast<alicaTests::TestWorldModel*>(acs[0]->getWorldModel());
    auto twm2 = dynamic_cast<alicaTests::TestWorldModel*>(acs[1]->getWorldModel());

    const Plan* plan = aes[0]->getPlanRepository().getPlans().find(1414403413451);
    ASSERT_NE(plan, nullptr) << "Plan 1414403413451 is unknown";
    ASSERT_NE(plan->getUtilityFunction(), nullptr) << "UtilityFunction is null!";
    auto uSummandAe = plan->getUtilityFunction()->getUtilSummands()[0].get();
    alica::DummyTestSummand* dbr = dynamic_cast<alica::DummyTestSummand*>(uSummandAe);
    dbr->robotId = acs[0]->getLocalAgentId();

    auto uSummandAe2 = aes[1]->getPlanRepository().getPlans().find(1414403413451)->getUtilityFunction()->getUtilSummands()[0].get();
    alica::DummyTestSummand* dbr2 = dynamic_cast<alica::DummyTestSummand*>(uSummandAe2);
    dbr2->robotId = acs[1]->getLocalAgentId();

    AgentId id1 = acs[0]->getLocalAgentId();
    AgentId id2 = acs[1]->getLocalAgentId();
    ASSERT_NE(id1, id2) << "Agents use the same ID.";

    aes[0]->start();
    aes[1]->start();

    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    twm1->robotsXPos.push_back(0);
    twm1->robotsXPos.push_back(2000);

    twm2->robotsXPos.push_back(2000);
    twm2->robotsXPos.push_back(0);

    for (int i = 0; i < 21; i++) {
        acs[0]->stepEngine();
        acs[1]->stepEngine();

        if (i == 1) {
            EXPECT_TRUE(alica::test::Util::isStateActive(aes[0], 1414403553717));
            EXPECT_TRUE(alica::test::Util::isStateActive(aes[1], 1414403553717));
        }

        if (i == 20) {
            EXPECT_TRUE(alica::test::Util::isStateActive(aes[0], 1414403553717));
            EXPECT_TRUE(alica::test::Util::isStateActive(aes[1], 1414403429950));
        }
    }

#if 0
    // this won't work Luca
    // ask to Veeraj
    auto logs = twm1->tracingLogs;

    bool foundTaskAssignmentChangeLog = false;
    for (auto log : logs) {
        std::cerr<<log.first<<std::endl;
        if (log.first == "TaskAssignmentChange") {
            foundTaskAssignmentChangeLog = true;
            break;
        }
    }
    EXPECT_TRUE(foundTaskAssignmentChangeLog);
#endif
    EXPECT_EQ(twm1->tracingParents["EmptyBehaviour"], "AuthorityTest");
    EXPECT_EQ(twm2->tracingParents["EmptyBehaviour"], "AuthorityTest"); // added by Luca
}

} // namespace
} // namespace alica
