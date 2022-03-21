#include "test_alica.h"

#include "DynamicTaskBehavior.h"
#include "alica_tests/TaskInstantiationIntegrationWorldModel.h"

#include <alica/test/Util.h>
#include <engine/AlicaContext.h>
#include <engine/RunningPlan.h>
#include <engine/model/AlicaElement.h>
#include <engine/model/Behaviour.h>
#include <engine/model/EntryPoint.h>
#include <engine/model/Plan.h>
#include <engine/model/State.h>
#include <engine/model/Task.h>
#include <engine/util/HashFunctions.h>

#include <gtest/gtest.h>

namespace alica
{
namespace
{
// GTEST_FILTER=AlicaDynamicTaskPlanTest.* catkin run_tests -i alica_tests
class AlicaDynamicTaskPlanTest : public AlicaTestMultiAgentFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "DynamicTaskAssignmentTestMaster"; }

    const int agentCount = 2;
    int getAgentCount() const override { return agentCount; }
    const char* getHostName(int agentNumber) const override
    {
        if (agentNumber) {
            return "hairy";
        } else {
            return "nase";
        }
    }

    void startAgents()
    {
        for (uint8_t agent_index = 0; agent_index < getAgentCount(); agent_index++) {
            aes[agent_index]->start();
        }
    }

    void stepAgents()
    {
        for (uint8_t agent_index = 0; agent_index < getAgentCount(); agent_index++) {
            acs[agent_index]->stepEngine();
        }
    }

    void enableTransitionConditionToStart(int agentIdx)
    {
        if (agentIdx == 0) {
            alicaTests::TestWorldModel::curAgent(acs[0]->getLocalAgentId());
            alicaTests::TestWorldModel::getOne()->setTransitionCondition4496654201854254411(true);
        } else if (agentIdx == 1) {
            alicaTests::TestWorldModel::curAgent(acs[1]->getLocalAgentId());
            alicaTests::TestWorldModel::getTwo()->setTransitionCondition4496654201854254411(true);
        } else {
            ASSERT_FALSE(true) << "Invalid agentIdx";
        }
    }

    void enableTransitionConditionToTogetherPlan(int agentIdx)
    {
        if (agentIdx == 0) {
            alicaTests::TestWorldModel::curAgent(acs[0]->getLocalAgentId());
            alicaTests::TestWorldModel::getOne()->setTransitionCondition3126176581533900616(true);
        } else if (agentIdx == 1) {
            alicaTests::TestWorldModel::curAgent(acs[1]->getLocalAgentId());
            alicaTests::TestWorldModel::getTwo()->setTransitionCondition3126176581533900616(true);
        } else {
            ASSERT_FALSE(true) << "Invalid agentIdx";
        }
    }

    void enableTransitionConditionToSuccess(int agentIdx)
    {
        if (agentIdx == 0) {
            alicaTests::TestWorldModel::curAgent(acs[0]->getLocalAgentId());
            alicaTests::TestWorldModel::getOne()->setTransitionCondition1078898265232036813(true);
        } else if (agentIdx == 1) {
            alicaTests::TestWorldModel::curAgent(acs[1]->getLocalAgentId());
            alicaTests::TestWorldModel::getTwo()->setTransitionCondition1078898265232036813(true);
        } else {
            ASSERT_FALSE(true) << "Invalid agentIdx";
        }
    }

    void enableTransitionConditionToFinish(int agentIdx)
    {
        if (agentIdx == 0) {
            alicaTests::TestWorldModel::curAgent(acs[0]->getLocalAgentId());
            alicaTests::TestWorldModel::getOne()->setTransitionToFinish(true);
        } else {
            alicaTests::TestWorldModel::curAgent(acs[1]->getLocalAgentId());
            alicaTests::TestWorldModel::getTwo()->setTransitionToFinish(true);
        }
    }

    static void checkAlicaElement(const alica::AlicaElement* alicaElement, long id, std::string name)
    {
        EXPECT_EQ(id, alicaElement->getId());
        EXPECT_STREQ(name.c_str(), alicaElement->getName().c_str());
    }

    static void checkEntryPoint(const alica::EntryPoint* ep, long id, std::string name, bool successRequired, int minCardinality, int maxCardinality,
            long stateID, long taskID, std::string taskName)
    {
        checkAlicaElement(ep, id, name);
        EXPECT_EQ(successRequired, ep->isSuccessRequired());
        EXPECT_EQ(minCardinality, ep->getMinCardinality());
        EXPECT_EQ(maxCardinality, ep->getMaxCardinality());
        EXPECT_EQ(stateID, ep->getState()->getId());
        EXPECT_EQ(taskID, ep->getTask()->getId());
        EXPECT_STREQ(taskName.c_str(), ep->getTask()->getName().c_str());
    }

    static constexpr int64_t kMasterPlanInitStateId = 4467904887554008050;
    static constexpr int64_t kMasterPlanStartStateId = 751302000461175045;
    static constexpr int64_t kDynamicState1Id = 2800951832651805821;
    static constexpr int64_t kDynamicTaskAssignmentPlanId = 2252865124432942907;
    static constexpr int64_t kDynamicTaskBehaviorId = 4044546549214673470;
    static constexpr int64_t kDynamicTaskEntrypointId = 3150793708487666867;
    static constexpr int64_t kDynamicTaskId = 1163169622598227531;

    static constexpr int64_t kDynamicTaskLCEntrypointId = 3626583666892196532;
    static constexpr int64_t kDynamicTaskBehaviourLDStateId = 3534468625273851172;

    static constexpr const char* kDynamicTaskName = "DynamicTask";
};

// GTEST_FILTER=AlicaTaskInstantiationIntegrationTest.* catkin run_tests -i alica_tests

class AlicaTaskInstantiationIntegrationTest : public AlicaTestMultiAgentFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "TaskInstantiationIntegrationTestMaster"; }
    int getAgentCount() const override { return 4; }
    int getPayloadCount() const { return 8; }

    const char* getHostName(int agentNumber) const override
    {
        switch(agentNumber) {
            case 0: return "nase";
            case 1: return "hairy";
            case 2: return "fred";
            case 3: return "george";
            default: return "";
        }
    }

    void startAgents()
    {
        for (uint8_t agent_index = 0; agent_index < getAgentCount(); agent_index++) {
            aes[agent_index]->start();
        }
    }

    void stepAgents()
    {
        for (uint8_t agent_index = 0; agent_index < getAgentCount(); agent_index++) {
            acs[agent_index]->stepEngine();
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
        alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(), std::make_unique<alica::PlanCreator>());

        for (int i = 0; i < getAgentCount(); ++i) {
            alica::AlicaContext* ac =
                    new alica::AlicaContext(alica::AlicaContextParams(getHostName(i), path + "/etc/", getRoleSetName(), getMasterPlanName(), stepEngine()));
            ASSERT_TRUE(ac->isValid());
            cbQueues.emplace_back(std::make_unique<ros::CallbackQueue>());
            spinners.emplace_back(std::make_unique<ros::AsyncSpinner>(4, cbQueues.back().get()));
            ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
            ac->setWorldModel<alicaTests::TaskInstantiationIntegrationWorldModel>();
            ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>(*cbQueues.back());
            alica::AlicaEngine* ae = AlicaTestsEngineGetter::getEngine(ac);
            const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
            spinners.back()->start();
            EXPECT_TRUE(ae->init(creators));
            acs.push_back(ac);
            aes.push_back(ae);
        }

        wms = make_shared<std::vector<alicaTests::TaskInstantiationIntegrationWorldModel*>>();
        for (int i = 0; i < getAgentCount(); i++) {
            IAlicaWorldModel* wmTemp = acs[i]->getWorldModel();            
            alicaTests::TaskInstantiationIntegrationWorldModel* wm = dynamic_cast<alicaTests::TaskInstantiationIntegrationWorldModel*>(wmTemp);
            wms->push_back(wm);
            wm->wms = wms;
            wm->agentId = aes[i]->getTeamManager().getLocalAgentID();
        }
    }

    std::shared_ptr<std::vector<alicaTests::TaskInstantiationIntegrationWorldModel*>> wms;
    std::vector<alicaTests::Payload> payloads;
    std::unordered_map<uint64_t, std::pair<uint64_t, uint64_t>> agentLocations;
    const uint64_t TP_TO_PICK_SPOT = 2867928428650937962;
};

TEST_F(AlicaTaskInstantiationIntegrationTest, taskInstantiationIntegrationTest)
{
    ASSERT_NO_SIGNAL

    for (int i = 0; i < getPayloadCount(); i++) {
        alicaTests::Payload payload;
        payload.id = i;
        payload.state = alicaTests::PayloadState::READY_FOR_PICKUP;
        payload.pickX = i * 50;
        payload.pickY = i * 50;
        payload.dropX = i * 50 + 10;
        payload.dropY = i * 50 + 10;

        payloads.push_back(payload);
    }

    for (int i = 0; i < getAgentCount(); i++) {
        agentLocations[i + 8] = std::pair<uint64_t, uint64_t>(0, 0);
    }

    for (int i = 0; i < getAgentCount(); i++) {
        wms->at(i)->payloads = payloads;
        wms->at(i)->agentLocations = agentLocations;
    }

    startAgents();

    bool allPayloadsAreDropped = false;
    int count = 0;
    while (!allPayloadsAreDropped && count < 10) {
        stepAgents();
        count++;
        aes[0]->getAlicaClock().sleep(alica::AlicaTime::milliseconds(500));
        payloads = wms->at(0)->payloads;
        allPayloadsAreDropped = true;

        for (alicaTests::Payload payload : payloads) {
            if (payload.state != alicaTests::PayloadState::DROPPED) {
                allPayloadsAreDropped = false;
            }
        }
    }

    ASSERT_TRUE(allPayloadsAreDropped);
}

TEST_F(AlicaDynamicTaskPlanTest, testMultipleEntrypoints)
{
    // Plan initially has no EP
    const Plan* plan[2];
    plan[0] = aes[0]->getPlanRepository().getPlans().find(kDynamicTaskAssignmentPlanId);
    plan[1] = aes[1]->getPlanRepository().getPlans().find(kDynamicTaskAssignmentPlanId);
    EXPECT_EQ(0u, plan[0]->getEntryPoints().size()) << "Number of dynamic EntryPoints should still be empty.";
    EXPECT_EQ(0u, plan[1]->getEntryPoints().size()) << "Number of dynamic EntryPoints should still be empty.";

    // Make agents enter dynamic tasks
    ASSERT_NO_SIGNAL
    startAgents();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    stepAgents();
    enableTransitionConditionToStart(0);
    stepAgents();
    enableTransitionConditionToStart(1);
    stepAgents();

    // Check that plan now has a dynamic EP
    for (uint8_t agent_index = 0; agent_index < getAgentCount(); agent_index++) {
        ASSERT_TRUE(alica::test::Util::isStateActive(aes[agent_index], kMasterPlanStartStateId));
        ASSERT_TRUE(alica::test::Util::isPlanActive(aes[agent_index], kDynamicTaskAssignmentPlanId));
        ASSERT_TRUE(alica::test::Util::isStateActive(aes[agent_index], kDynamicState1Id));

        EXPECT_EQ(1u, plan[agent_index]->getEntryPoints().size()) << "A single dynamic EntryPoint should have been created by this point";

        const alica::EntryPoint* ep = plan[agent_index]->getEntryPoints().front();
        checkEntryPoint(ep, kDynamicTaskEntrypointId, "", false, 0, INT_MAX, kDynamicState1Id, kDynamicTaskId, kDynamicTaskName);
        EXPECT_EQ(ep->getDynamicId(), 1);
    }
}

/**
 * Tests whether it is possible to instantiate multiple tasks, dynamically, at runtime
 */
TEST_F(AlicaDynamicTaskPlanTest, runDynamicTasks)
{
    ASSERT_NO_SIGNAL
    startAgents();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    std::cout << "AE1 (" << acs[0]->getLocalAgentId() << ")" << std::endl;
    std::cout << "AE2 (" << acs[1]->getLocalAgentId() << ")" << std::endl;

    stepAgents();
    for (uint8_t agent_index = 0; agent_index < getAgentCount(); agent_index++) {
        ASSERT_TRUE(alica::test::Util::isStateActive(aes[agent_index], kMasterPlanInitStateId));
    }

    enableTransitionConditionToStart(0);
    stepAgents();
    enableTransitionConditionToStart(1);
    stepAgents();

    for (uint8_t agent_index = 0; agent_index < getAgentCount(); agent_index++) {
        ASSERT_TRUE(alica::test::Util::isStateActive(aes[agent_index], kMasterPlanStartStateId));
        ASSERT_TRUE(alica::test::Util::isPlanActive(aes[agent_index], kDynamicTaskAssignmentPlanId));
        ASSERT_TRUE(alica::test::Util::isStateActive(aes[agent_index], kDynamicState1Id));

        ASSERT_EQ(aes[agent_index]->getPlanBase().getDeepestNode()->getActiveEntryPoint()->getId(), kDynamicTaskLCEntrypointId);
        ASSERT_EQ(std::dynamic_pointer_cast<alica::DynamicTaskBehavior>(alica::test::Util::getBasicBehaviour(aes[agent_index], kDynamicTaskBehaviorId, 0))
                          ->callCounter,
                1);
    }
}

/**
 * Tests whether plan serialization works fine
 */
TEST_F(AlicaDynamicTaskPlanTest, serialize)
{
    // Make agents enter dynamic tasks
    ASSERT_NO_SIGNAL
    startAgents();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    stepAgents();
    enableTransitionConditionToStart(0);
    stepAgents();
    enableTransitionConditionToStart(1);
    stepAgents();

    // Verify the simple plan tree serialization
    IdGrp plan_trees[getAgentCount()];
    for (uint8_t agent_index = 0; agent_index < getAgentCount(); agent_index++) {
        ASSERT_TRUE(alica::test::Util::isStateActive(aes[agent_index], kDynamicState1Id));
        ASSERT_TRUE(alica::test::Util::isStateActive(aes[agent_index], kDynamicTaskBehaviourLDStateId));

        const RunningPlan* rpRoot = aes[agent_index]->getPlanBase().getRootNode();
        ASSERT_TRUE(rpRoot);

        // Build current Plan Tree
        IdGrp msg_to_send;
        int deepestNode = 0;
        int treeDepth = 0;
        rpRoot->toMessage(msg_to_send, rpRoot, deepestNode, treeDepth);
        plan_trees[agent_index] = msg_to_send;
    }
    std::vector<int64_t> expectedTree = {
            0, 751302000461175045, 2, 1633421497783210879, 3, 2765772942388464345, 0, 3534468625273851172, -1, -1, -1, 1, 2800951832651805821, -1, -1};
    ASSERT_EQ(plan_trees[0], expectedTree);
    ASSERT_EQ(plan_trees[1], expectedTree);
}

/**
 * Tests whether plan deserialization works fine
 */
TEST_F(AlicaDynamicTaskPlanTest, deserialize)
{
    // Make agents enter dynamic tasks
    ASSERT_NO_SIGNAL
    startAgents();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    stepAgents();
    enableTransitionConditionToStart(0);
    stepAgents();
    enableTransitionConditionToStart(1);
    stepAgents();

    auto ep0 = aes[1]->getPlanBase().getRootNode()->getChildren()[1]->getAssignment().getEntryPointOfAgent(acs[0]->getLocalAgentId());
    ASSERT_TRUE(ep0);
    checkEntryPoint(ep0, kDynamicTaskEntrypointId, "", false, 0, INT_MAX, kDynamicState1Id, kDynamicTaskId, kDynamicTaskName);
    ASSERT_TRUE(ep0->isDynamic());
    ASSERT_EQ(ep0->getDynamicId(), 1);

    auto ep1 = aes[0]->getPlanBase().getRootNode()->getChildren()[1]->getAssignment().getEntryPointOfAgent(acs[1]->getLocalAgentId());
    ASSERT_TRUE(ep1);
    checkEntryPoint(ep1, kDynamicTaskEntrypointId, "", false, 0, INT_MAX, kDynamicState1Id, kDynamicTaskId, kDynamicTaskName);
    ASSERT_TRUE(ep1->isDynamic());
    ASSERT_EQ(ep1->getDynamicId(), 1);
}

/**
 * Tests whether plan serialization works fine for multiple entrypoint plans
 */
TEST_F(AlicaDynamicTaskPlanTest, serializeMultipleEntryPoint)
{
    // Make agents enter dynamic tasks
    ASSERT_NO_SIGNAL
    startAgents();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    stepAgents();
    enableTransitionConditionToTogetherPlan(0);
    stepAgents();
    enableTransitionConditionToTogetherPlan(1);
    stepAgents();

    // Verify the simple plan tree serialization
    IdGrp plan_trees[getAgentCount()];
    for (uint8_t agent_index = 0; agent_index < getAgentCount(); agent_index++) {

        const RunningPlan* rpRoot = aes[agent_index]->getPlanBase().getRootNode();
        ASSERT_TRUE(rpRoot);

        // Build current Plan Tree
        IdGrp msg_to_send;
        int deepestNode = 0;
        int treeDepth = 0;
        rpRoot->toMessage(msg_to_send, rpRoot, deepestNode, treeDepth);
        plan_trees[agent_index] = msg_to_send;
    }
    std::vector<int64_t> expectedTree0 = {
            0, 3235149896384117046, 11, 2564904534754645793, 2, 1633421497783210879, 3, 2765772942388464345, 0, 3534468625273851172, -1, -1, -1, -1, -1};
    std::vector<int64_t> expectedTree1 = {0, 3235149896384117046, 22, 2362235348110947949, 1, 2800951832651805821, -1, -1, -1};
    ASSERT_EQ(plan_trees[0], expectedTree0);
    ASSERT_EQ(plan_trees[1], expectedTree1);
}

/**
 * Tests whether plan deserialization works fine
 */
TEST_F(AlicaDynamicTaskPlanTest, deserializeMultiTask)
{
    // Make agents enter dynamic tasks
    ASSERT_NO_SIGNAL
    startAgents();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    stepAgents();
    enableTransitionConditionToTogetherPlan(0);
    stepAgents();
    enableTransitionConditionToTogetherPlan(1);
    stepAgents();

    // agentId: 0 -> nase (defender), 1 -> hairy (attacker)
    auto defId = acs[0]->getLocalAgentId();
    auto attId = acs[1]->getLocalAgentId();

    const auto& defAssignment = aes[0]->getPlanBase().getRootNode()->getChildren()[0]->getAssignment();
    const auto& attAssignment = aes[1]->getPlanBase().getRootNode()->getChildren()[0]->getAssignment();

    auto defEpId = 2665027307523422046;
    auto defDynamicEpId = 11;
    auto defStateId = 2564904534754645793;
    auto attEpId = 2633712961224790694;
    auto attDynamicEpId = 22;
    auto attStateId = 2362235348110947949;

    auto epOfAttAsPerDef = defAssignment.getEntryPointOfAgent(attId);
    ASSERT_TRUE(epOfAttAsPerDef);
    ASSERT_EQ(epOfAttAsPerDef->getId(), attEpId);
    ASSERT_TRUE(epOfAttAsPerDef->isDynamic());
    ASSERT_EQ(epOfAttAsPerDef->getDynamicId(), attDynamicEpId);

    auto epOfDefAsPerAtt = attAssignment.getEntryPointOfAgent(defId);
    ASSERT_TRUE(epOfDefAsPerAtt);
    ASSERT_EQ(epOfDefAsPerAtt->getId(), defEpId);
    ASSERT_TRUE(epOfDefAsPerAtt->isDynamic());
    ASSERT_EQ(epOfDefAsPerAtt->getDynamicId(), defDynamicEpId);

    auto epOfDefAsPerDef = defAssignment.getEntryPointOfAgent(defId);
    ASSERT_TRUE(epOfDefAsPerDef);
    ASSERT_EQ(epOfDefAsPerDef->getId(), defEpId);
    ASSERT_TRUE(epOfDefAsPerDef->isDynamic());
    ASSERT_EQ(epOfDefAsPerDef->getDynamicId(), defDynamicEpId);

    auto epOfAttAsPerAtt = attAssignment.getEntryPointOfAgent(attId);
    ASSERT_TRUE(epOfAttAsPerAtt);
    ASSERT_EQ(epOfAttAsPerAtt->getId(), attEpId);
    ASSERT_TRUE(epOfAttAsPerAtt->isDynamic());
    ASSERT_EQ(epOfAttAsPerAtt->getDynamicId(), attDynamicEpId);

    auto stateOfAttAsPerDef = defAssignment.getStateOfAgent(attId);
    ASSERT_TRUE(stateOfAttAsPerDef);
    ASSERT_EQ(stateOfAttAsPerDef->getId(), attStateId);

    auto stateOfDefAsPerAtt = attAssignment.getStateOfAgent(defId);
    ASSERT_TRUE(stateOfDefAsPerAtt);
    ASSERT_EQ(stateOfDefAsPerAtt->getId(), defStateId);

    auto stateOfDefAsPerDef = defAssignment.getStateOfAgent(defId);
    ASSERT_TRUE(stateOfDefAsPerDef);
    ASSERT_EQ(stateOfDefAsPerDef->getId(), defStateId);

    auto stateOfAttAsPerAtt = attAssignment.getStateOfAgent(attId);
    ASSERT_TRUE(stateOfAttAsPerAtt);
    ASSERT_EQ(stateOfAttAsPerAtt->getId(), attStateId);
}

/**
 * Tests whether the success marks are sent out correctly
 */
TEST_F(AlicaDynamicTaskPlanTest, successMarkOuput)
{
    ASSERT_NO_SIGNAL
    startAgents();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    stepAgents();

    // Execute DynamicTaskAssignmentTestPlan in different branches
    enableTransitionConditionToStart(0);
    stepAgents();
    ASSERT_TRUE(alica::test::Util::isStateActive(aes[0], 751302000461175045));

    enableTransitionConditionToTogetherPlan(1);
    stepAgents();
    ASSERT_TRUE(alica::test::Util::isStateActive(aes[1], 3235149896384117046));

    // Initially nothing has succeeded
    auto successMarks0 = aes[0]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
    auto successMarks1 = aes[1]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();

    ASSERT_TRUE(successMarks0.empty());
    ASSERT_TRUE(successMarks1.empty());

    uint64_t epId = 3150793708487666867;
    uint64_t epDynamicId = 1;

    uint64_t naseParentDynamicEpId = 0;
    uint64_t naseParentStateId = 751302000461175045;

    // Nase succeeds
    enableTransitionConditionToSuccess(0);
    stepAgents();
    successMarks0 = aes[0]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
    decltype(successMarks0) expectedMarks0 = {
            hashCombine(contextHash(0), hashCombine(contextHash(naseParentDynamicEpId), contextHash(naseParentStateId))), epId, epDynamicId};
    ASSERT_EQ(successMarks0, expectedMarks0);

    uint64_t hairyGrandParentDynamicEpId = naseParentDynamicEpId;
    uint64_t hairyGrandParentStateId = 3235149896384117046;
    uint64_t hairyParentDynamicEpId = 22;
    uint64_t hairyParentStateId = 2362235348110947949;

    // Hairy succeeds
    enableTransitionConditionToSuccess(1);
    stepAgents();
    successMarks1 = aes[1]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
    decltype(successMarks1) expectedMarks1 = {
            hashCombine(hashCombine(contextHash(0), hashCombine(contextHash(hairyGrandParentDynamicEpId), contextHash(hairyGrandParentStateId))),
                    hashCombine(contextHash(hairyParentDynamicEpId), contextHash(hairyParentStateId))),
            epId, epDynamicId};
    ASSERT_EQ(successMarks1, expectedMarks1);
}

/**
 * Test wether successMarks are cleared when a parentContext has no active agents in it
 */
TEST_F(AlicaDynamicTaskPlanTest, clearSuccessMarkOnEmptyPlanContext)
{
    ASSERT_NO_SIGNAL
    startAgents();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    stepAgents();

    // Execute DynamicTaskAssignmentTestPlan in one branch
    enableTransitionConditionToStart(0);
    stepAgents();

    // Initially nothing has succeeded
    auto successMarks0 = aes[0]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
    auto successMarks1 = aes[1]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();

    ASSERT_TRUE(successMarks0.empty());
    ASSERT_TRUE(successMarks1.empty());

    // nase succeeds
    enableTransitionConditionToSuccess(0);
    stepAgents();
    successMarks0 = aes[0]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
    successMarks1 = aes[1]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
    EXPECT_GT(successMarks0.size(), 0u);
    ASSERT_TRUE(successMarks1.empty());

    auto& defAssignment = aes[0]->getPlanBase().getRootNode()->getChildren()[1]->editAssignment();

    // Assignment of nase has success stored
    EXPECT_EQ(defAssignment.editSuccessData().getRaw()[0][0], aes[0]->getTeamManager().getLocalAgentID());

    // nase leaves planContext, successMarks should be cleared, because planContext is empty now
    enableTransitionConditionToFinish(0);
    stepAgents();

    successMarks0 = aes[0]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
    successMarks1 = aes[1]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
    ASSERT_TRUE(successMarks0.empty());
    ASSERT_TRUE(successMarks1.empty());

    // Execute DynamicTaskAssignmentTestPlan in the same branch
    enableTransitionConditionToStart(1);
    stepAgents();

    // hairy succeeds
    enableTransitionConditionToSuccess(1);
    stepAgents();

    auto& attAssignment = aes[1]->getPlanBase().getRootNode()->getChildren()[1]->editAssignment();

    // hairy should be the only agent with a success in this context, nases success should have been cleared
    EXPECT_EQ(attAssignment.editSuccessData().getRaw()[0].size(), 1u);
    EXPECT_EQ(attAssignment.editSuccessData().getRaw()[0][0], aes[1]->getTeamManager().getLocalAgentID());
}

/**
 * Tests wether own successMarks are correctly added to assignments
 */
TEST_F(AlicaDynamicTaskPlanTest, ownSuccessMarksInAssignments)
{
    ASSERT_NO_SIGNAL
    startAgents();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    stepAgents();

    // Execute DynamicTaskAssignmentTestPlan in different branches
    enableTransitionConditionToStart(0);
    stepAgents();
    ASSERT_TRUE(alica::test::Util::isStateActive(aes[0], 751302000461175045));

    enableTransitionConditionToTogetherPlan(1);
    stepAgents();
    ASSERT_TRUE(alica::test::Util::isStateActive(aes[1], 3235149896384117046));

    // Initially nothing has succeeded
    auto successMarks0 = aes[0]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
    auto successMarks1 = aes[1]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();

    ASSERT_TRUE(successMarks0.empty());
    ASSERT_TRUE(successMarks1.empty());

    // nase succeeds
    enableTransitionConditionToSuccess(0);
    stepAgents();
    successMarks0 = aes[0]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
    successMarks1 = aes[1]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
    EXPECT_GT(successMarks0.size(), 0u);
    ASSERT_TRUE(successMarks1.empty());

    auto& defAssignment = aes[0]->getPlanBase().getRootNode()->getChildren()[1]->editAssignment();
    auto& attAssignment = aes[1]->getPlanBase().getRootNode()->getChildren()[0]->getChildren()[0]->editAssignment();

    // Assignment of nase has success stored
    EXPECT_EQ(defAssignment.editSuccessData().getRaw()[0][0], aes[0]->getTeamManager().getLocalAgentID());
    // Assignment of hairy has no success stored because it is in another parentContext
    EXPECT_FALSE(attAssignment.isAnyTaskSuccessful());

    // hairy succeeds
    enableTransitionConditionToSuccess(1);
    stepAgents();
    successMarks0 = aes[0]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
    successMarks1 = aes[1]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
    EXPECT_GT(successMarks0.size(), 0u);
    EXPECT_GT(successMarks1.size(), 0u);

    defAssignment = aes[0]->getPlanBase().getRootNode()->getChildren()[1]->editAssignment();
    attAssignment = aes[1]->getPlanBase().getRootNode()->getChildren()[0]->getChildren()[0]->editAssignment();
    // Assignment of nase has success stored
    EXPECT_EQ(defAssignment.editSuccessData().getRaw()[0][0], aes[0]->getTeamManager().getLocalAgentID());
    // Assignment of hairy now has success stored because it succeeded in another parentContext
    EXPECT_EQ(attAssignment.editSuccessData().getRaw()[0][0], aes[1]->getTeamManager().getLocalAgentID());

    enableTransitionConditionToFinish(1);
    stepAgents();
    successMarks0 = aes[0]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
    successMarks1 = aes[1]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
}

/**
 * Tests wether successMarks of other agents are correctly added to assignments
 */
TEST_F(AlicaDynamicTaskPlanTest, otherSuccessMarksInAssignments)
{
    ASSERT_NO_SIGNAL
    startAgents();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    stepAgents();

    // Execute DynamicTaskAssignmentTestPlan in different branches
    enableTransitionConditionToStart(0);
    stepAgents();

    enableTransitionConditionToStart(1);
    stepAgents();

    ASSERT_TRUE(alica::test::Util::isStateActive(aes[0], 751302000461175045));
    ASSERT_TRUE(alica::test::Util::isStateActive(aes[1], 751302000461175045));

    // Initially nothing has succeeded
    auto successMarks0 = aes[0]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
    auto successMarks1 = aes[1]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();

    ASSERT_TRUE(successMarks0.empty());
    ASSERT_TRUE(successMarks1.empty());

    // nase succeeds
    enableTransitionConditionToSuccess(0);
    stepAgents();
    successMarks0 = aes[0]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
    successMarks1 = aes[1]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
    EXPECT_GT(successMarks0.size(), 0u);
    ASSERT_TRUE(successMarks1.empty());

    auto& defAssignment = aes[0]->getPlanBase().getRootNode()->getChildren()[1]->editAssignment();
    auto& attAssignment = aes[1]->getPlanBase().getRootNode()->getChildren()[1]->editAssignment();

    // Assignment of nase has success stored
    EXPECT_EQ(defAssignment.editSuccessData().getRaw()[0][0], aes[0]->getTeamManager().getLocalAgentID());
    // Assignment of hairy has success stored because it is in the same parentContext
    EXPECT_EQ(attAssignment.editSuccessData().getRaw()[0][0], aes[0]->getTeamManager().getLocalAgentID());

    // hairy succeeds
    enableTransitionConditionToSuccess(1);
    stepAgents();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    successMarks0 = aes[0]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
    successMarks1 = aes[1]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
    EXPECT_GT(successMarks0.size(), 0u);
    EXPECT_GT(successMarks1.size(), 0u);

    attAssignment = aes[1]->getPlanBase().getRootNode()->getChildren()[1]->editAssignment();

    // Assignment of hairy has the success of both agents stored
    EXPECT_EQ(attAssignment.editSuccessData().getRaw()[0][0], aes[0]->getTeamManager().getLocalAgentID());
    EXPECT_EQ(attAssignment.editSuccessData().getRaw()[0][1], aes[1]->getTeamManager().getLocalAgentID());

    enableTransitionConditionToFinish(1);
    stepAgents();
    successMarks0 = aes[0]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();
    successMarks1 = aes[1]->getTeamManager().getLocalAgent()->getEngineData().getSuccessMarks().toMsg();

    // successMarks will not be cleared, because nase is still in the parentContext
    EXPECT_EQ(defAssignment.editSuccessData().getRaw()[0][1], aes[0]->getTeamManager().getLocalAgentID());
    EXPECT_EQ(defAssignment.editSuccessData().getRaw()[0][0], aes[1]->getTeamManager().getLocalAgentID());
}

} // namespace
} // namespace alica
