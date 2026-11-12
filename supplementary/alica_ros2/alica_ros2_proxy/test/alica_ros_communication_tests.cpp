#include <communication/AlicaRosCommunication.h>
#include <communication/AlicaRosEdgeBroadcastCommunication.h>

#include <alica_msgs/msg/agent_announcement_list.hpp>
#include <alica_msgs/msg/agent_query_list.hpp>
#include <alica_msgs/msg/allocation_authority_info_list.hpp>
#include <alica_msgs/msg/plan_tree_info_list.hpp>
#include <alica_msgs/msg/solver_result_list.hpp>
#include <alica_msgs/msg/sync_ready_list.hpp>
#include <alica_msgs/msg/sync_talk_list.hpp>

#include <engine/containers/AgentAnnouncement.h>
#include <engine/containers/AgentQuery.h>
#include <engine/containers/AllocationAuthorityInfo.h>
#include <engine/containers/PlanTreeInfo.h>
#include <engine/containers/SolverResult.h>
#include <engine/containers/SyncReady.h>
#include <engine/containers/SyncTalk.h>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace
{

using namespace std::chrono_literals;

bool waitFor(const std::function<bool()>& condition, std::chrono::milliseconds timeout = 5000ms)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (condition()) {
            return true;
        }
        std::this_thread::sleep_for(10ms);
    }
    return condition();
}

// Stores everything that the communication gives to the engine
class Received
{
public:
    alica::AlicaCommunicationHandlers handlers()
    {
        alica::AlicaCommunicationHandlers handlers;
        handlers.onSyncTalkHandler = [this](std::shared_ptr<alica::SyncTalk> st) { add(syncTalks, *st); };
        handlers.onSyncReadyHandler = [this](std::shared_ptr<alica::SyncReady> sr) { add(syncReadies, *sr); };
        handlers.incomingAuthorityMessageHandler = [this](const alica::AllocationAuthorityInfo& aai) { add(authorityInfos, aai); };
        handlers.planTreeInfohandler = [this](std::shared_ptr<alica::PlanTreeInfo> pti) { add(planTreeInfos, *pti); };
        handlers.onSolverResultHandler = [this](const alica::SolverResult& sr) { add(solverResults, sr); };
        handlers.agentQueryHandler = [this](const alica::AgentQuery& pq) { add(agentQueries, pq); };
        handlers.agentAnnouncementHandler = [this](const alica::AgentAnnouncement& pa) { add(agentAnnouncements, pa); };
        return handlers;
    }

    template <class T>
    size_t count(const std::vector<T>& received) const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return received.size();
    }

    std::vector<alica::SyncTalk> syncTalks;
    std::vector<alica::SyncReady> syncReadies;
    std::vector<alica::AllocationAuthorityInfo> authorityInfos;
    std::vector<alica::PlanTreeInfo> planTreeInfos;
    std::vector<alica::SolverResult> solverResults;
    std::vector<alica::AgentQuery> agentQueries;
    std::vector<alica::AgentAnnouncement> agentAnnouncements;

private:
    template <class T>
    void add(std::vector<T>& received, const T& value)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        received.push_back(value);
    }

    mutable std::mutex _mutex;
};

alica::AgentAnnouncement makeAnnouncement(alica::AgentId id)
{
    alica::AgentAnnouncement pa;
    pa.senderID = id;
    pa.token = 7;
    pa.senderName = "agent_" + std::to_string(id);
    pa.senderSdk = 3;
    pa.planHash = 42;
    pa.roleId = 5;
    pa.capabilities.emplace_back("speed", "fast");
    return pa;
}

class AlicaRosCommunicationTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        _node = rclcpp::Node::make_shared("alica_ros_communication_tests");
        _executor.add_node(_node);
        _spinThread = std::thread([this]() { _executor.spin(); });
    }

    void TearDown() override
    {
        _executor.cancel();
        _spinThread.join();
    }

    // Waits until each topic has at least this many subscriptions in the ROS graph
    static bool waitForMatch(const rclcpp::Node::SharedPtr& node, const std::vector<std::string>& topics, size_t subscriptions)
    {
        return waitFor([&]() {
            for (const auto& topic : topics) {
                if (node->count_subscribers(topic) < subscriptions) {
                    return false;
                }
            }
            return true;
        });
    }

    rclcpp::Node::SharedPtr _node;
    rclcpp::executors::MultiThreadedExecutor _executor;
    std::thread _spinThread;
};

const std::vector<std::string> alicaEngineTopics = {"/AlicaEngine/AllocationAuthorityInfo", "/AlicaEngine/PlanTreeInfo", "/AlicaEngine/SyncReady",
        "/AlicaEngine/SyncTalk", "/AlicaEngine/SolverResult", "/AlicaEngine/AgentQuery", "/AlicaEngine/AgentAnnouncement"};

TEST_F(AlicaRosCommunicationTest, AllMessageTypesRoundTrip)
{
    // Also for the senders: they subscribe to the topics they publish on
    Received ignored;
    Received received;
    alicaRosProxy::AlicaRosCommunication sender(ignored.handlers(), _node);
    alicaRosProxy::AlicaRosCommunication receiver(received.handlers(), _node);
    sender.startCommunication();
    receiver.startCommunication();
    // Both communications subscribe on the same node
    ASSERT_TRUE(waitForMatch(_node, alicaEngineTopics, 2));

    alica::AllocationAuthorityInfo aai;
    aai.senderID = 1;
    aai.planId = 10;
    aai.parentState = 11;
    aai.planType = 12;
    aai.authority = 2;
    alica::EntryPointRobots ep;
    ep.entrypoint = 13;
    ep.robots = {1, 2};
    aai.entryPointRobots.push_back(ep);
    sender.sendAllocationAuthority(aai);

    alica::PlanTreeInfo pti;
    pti.senderID = 1;
    pti.stateIDs = {20, 21};
    pti.succeededEPs = {22};
    sender.sendPlanTreeInfo(pti);

    alica::SyncReady sr;
    sr.senderID = 1;
    sr.synchronisationID = 30;
    sender.sendSyncReady(sr);

    alica::SyncTalk st;
    st.senderID = 1;
    alica::SyncData sd;
    sd.agentID = 2;
    sd.transitionID = 31;
    sd.conditionHolds = true;
    sd.ack = false;
    st.syncData.push_back(sd);
    sender.sendSyncTalk(st);

    alica::SolverResult solverResult;
    solverResult.senderID = 1;
    alica::SolverVar sv;
    sv.id = 40;
    sv.value = "abc";
    solverResult.vars.push_back(sv);
    sender.sendSolverResult(solverResult);

    alica::AgentQuery pq;
    pq.senderID = 1;
    pq.senderSdk = 3;
    pq.planHash = 42;
    sender.sendAgentQuery(pq);

    sender.sendAgentAnnouncement(makeAnnouncement(1));

    ASSERT_TRUE(waitFor([&]() {
        return received.count(received.authorityInfos) == 1 && received.count(received.planTreeInfos) == 1 && received.count(received.syncReadies) == 1 &&
               received.count(received.syncTalks) == 1 && received.count(received.solverResults) == 1 && received.count(received.agentQueries) == 1 &&
               received.count(received.agentAnnouncements) == 1;
    }));

    const auto& raai = received.authorityInfos.front();
    EXPECT_EQ(raai.senderID, 1u);
    EXPECT_EQ(raai.planId, 10);
    EXPECT_EQ(raai.parentState, 11);
    EXPECT_EQ(raai.planType, 12);
    EXPECT_EQ(raai.authority, 2u);
    ASSERT_EQ(raai.entryPointRobots.size(), 1u);
    EXPECT_EQ(raai.entryPointRobots.front().entrypoint, 13);
    EXPECT_EQ(raai.entryPointRobots.front().robots, (std::vector<alica::AgentId>{1, 2}));

    EXPECT_EQ(received.planTreeInfos.front().stateIDs, (std::vector<int64_t>{20, 21}));
    EXPECT_EQ(received.planTreeInfos.front().succeededEPs, (std::vector<int64_t>{22}));

    EXPECT_EQ(received.syncReadies.front().synchronisationID, 30);

    ASSERT_EQ(received.syncTalks.front().syncData.size(), 1u);
    EXPECT_EQ(received.syncTalks.front().syncData.front().agentID, 2u);
    EXPECT_EQ(received.syncTalks.front().syncData.front().transitionID, 31);
    EXPECT_TRUE(received.syncTalks.front().syncData.front().conditionHolds);
    EXPECT_FALSE(received.syncTalks.front().syncData.front().ack);

    ASSERT_EQ(received.solverResults.front().vars.size(), 1u);
    EXPECT_EQ(received.solverResults.front().vars.front().id, 40);
    EXPECT_EQ(received.solverResults.front().vars.front().value, "abc");

    EXPECT_EQ(received.agentQueries.front().planHash, 42u);

    const auto& rpa = received.agentAnnouncements.front();
    EXPECT_EQ(rpa.senderID, 1u);
    EXPECT_EQ(rpa.token, 7u);
    EXPECT_EQ(rpa.senderName, "agent_1");
    EXPECT_EQ(rpa.roleId, 5);
    ASSERT_EQ(rpa.capabilities.size(), 1u);
    EXPECT_EQ(rpa.capabilities.front(), std::make_pair(std::string("speed"), std::string("fast")));
}

TEST_F(AlicaRosCommunicationTest, NothingSentOrReceivedWhenStopped)
{
    // Also for the senders: they subscribe to the topics they publish on
    Received ignored;
    Received received;
    alicaRosProxy::AlicaRosCommunication sender(ignored.handlers(), _node);
    alicaRosProxy::AlicaRosCommunication receiver(received.handlers(), _node);
    ASSERT_TRUE(waitForMatch(_node, alicaEngineTopics, 2));

    // Sender stopped: nothing is published
    receiver.startCommunication();
    sender.sendAgentAnnouncement(makeAnnouncement(1));
    std::this_thread::sleep_for(300ms);
    EXPECT_EQ(received.count(received.agentAnnouncements), 0u);

    // Receiver stopped: the message arrives, but is not given to the engine
    receiver.stopCommunication();
    sender.startCommunication();
    alica::PlanTreeInfo pti;
    pti.senderID = 1;
    sender.sendPlanTreeInfo(pti);
    std::this_thread::sleep_for(300ms);
    EXPECT_EQ(received.count(received.planTreeInfos), 0u);
}

TEST_F(AlicaRosCommunicationTest, LateJoinerGetsLastAnnouncementAndQuery)
{
    // Also for the senders: they subscribe to the topics they publish on
    Received ignored;
    // The ROS1 proxy latched these topics. An agent that starts later must still see the agents that started before it.
    alicaRosProxy::AlicaRosCommunication early(ignored.handlers(), _node);
    early.startCommunication();
    early.sendAgentQuery(alica::AgentQuery());
    early.sendAgentAnnouncement(makeAnnouncement(1));

    // This shows that a running communication gets messages that were published before its subscriptions matched,
    // which is what lets running agents see the query and announcement of an agent that just started.
    // The late agent has its own node, which spins only after startCommunication(). On a node that already spins, the
    // stored messages can arrive between the constructor and startCommunication(), and the communication drops them.
    auto lateNode = rclcpp::Node::make_shared("alica_ros_communication_tests_late");
    Received received;
    alicaRosProxy::AlicaRosCommunication late(received.handlers(), lateNode);
    late.startCommunication();
    _executor.add_node(lateNode);
    const bool gotBoth = waitFor([&]() { return received.count(received.agentAnnouncements) == 1 && received.count(received.agentQueries) == 1; });
    _executor.remove_node(lateNode);
    ASSERT_TRUE(gotBoth);
    EXPECT_EQ(received.agentAnnouncements.front().senderID, 1u);
}

TEST_F(AlicaRosCommunicationTest, DefaultConstructedCommunicationsOwnUniqueNodes)
{
    // Also for the senders: they subscribe to the topics they publish on
    Received ignored;
    Received received;
    alicaRosProxy::AlicaRosCommunication first(ignored.handlers());
    alicaRosProxy::AlicaRosCommunication second(received.handlers());
    first.startCommunication();
    second.startCommunication();
    ASSERT_TRUE(waitForMatch(_node, alicaEngineTopics, 2));

    size_t ownNodes = 0;
    for (const auto& name : _node->get_node_names()) {
        if (name.find("alica_communication_") != std::string::npos) {
            ++ownNodes;
        }
    }
    EXPECT_EQ(ownNodes, 2u);

    // Spun by the internal executor
    alica::PlanTreeInfo pti;
    pti.senderID = 1;
    first.sendPlanTreeInfo(pti);
    ASSERT_TRUE(waitFor([&]() { return received.count(received.planTreeInfos) == 1; }));
}

TEST_F(AlicaRosCommunicationTest, EdgeBroadcastReceivesEveryListElement)
{
    Received received;
    alicaRosProxy::AlicaRosEdgeBroadcastCommunication communication(received.handlers(), _node);
    communication.startCommunication();

    auto broadcaster = rclcpp::Node::make_shared("edge_broadcaster");
    const auto qos = rclcpp::QoS(10).reliable();
    auto announcementPub = broadcaster->create_publisher<alica_msgs::msg::AgentAnnouncementList>("/edge_broadcaster/AgentAnnouncements", qos);
    auto queryPub = broadcaster->create_publisher<alica_msgs::msg::AgentQueryList>("/edge_broadcaster/AgentQueries", qos);
    auto authorityPub = broadcaster->create_publisher<alica_msgs::msg::AllocationAuthorityInfoList>("/edge_broadcaster/AllocationAuthorityInfos", qos);
    auto planTreePub = broadcaster->create_publisher<alica_msgs::msg::PlanTreeInfoList>("/edge_broadcaster/PlanTreeInfos", qos);
    auto solverPub = broadcaster->create_publisher<alica_msgs::msg::SolverResultList>("/edge_broadcaster/SolverResults", qos);
    auto syncReadyPub = broadcaster->create_publisher<alica_msgs::msg::SyncReadyList>("/edge_broadcaster/SyncReadies", qos);
    auto syncTalkPub = broadcaster->create_publisher<alica_msgs::msg::SyncTalkList>("/edge_broadcaster/SyncTalks", qos);
    ASSERT_TRUE(waitFor([&]() {
        return announcementPub->get_subscription_count() == 1 && queryPub->get_subscription_count() == 1 && authorityPub->get_subscription_count() == 1 &&
               planTreePub->get_subscription_count() == 1 && solverPub->get_subscription_count() == 1 && syncReadyPub->get_subscription_count() == 1 &&
               syncTalkPub->get_subscription_count() == 1;
    }));

    alica_msgs::msg::AgentAnnouncementList announcements;
    announcements.agent_announcements.resize(2);
    announcements.agent_announcements[0].sender_id = 1;
    announcements.agent_announcements[1].sender_id = 2;
    announcementPub->publish(announcements);

    alica_msgs::msg::AgentQueryList queries;
    queries.agent_queries.resize(2);
    queryPub->publish(queries);

    alica_msgs::msg::AllocationAuthorityInfoList authorities;
    authorities.allocation_authority_infos.resize(2);
    authorityPub->publish(authorities);

    alica_msgs::msg::PlanTreeInfoList planTrees;
    planTrees.plan_tree_infos.resize(2);
    planTrees.plan_tree_infos[1].state_ids = {5};
    planTreePub->publish(planTrees);

    alica_msgs::msg::SolverResultList solverResults;
    solverResults.solver_results.resize(2);
    solverPub->publish(solverResults);

    alica_msgs::msg::SyncReadyList syncReadies;
    syncReadies.sync_readies.resize(2);
    syncReadyPub->publish(syncReadies);

    alica_msgs::msg::SyncTalkList syncTalks;
    syncTalks.sync_talks.resize(2);
    syncTalkPub->publish(syncTalks);

    ASSERT_TRUE(waitFor([&]() {
        return received.count(received.agentAnnouncements) == 2 && received.count(received.agentQueries) == 2 && received.count(received.authorityInfos) == 2 &&
               received.count(received.planTreeInfos) == 2 && received.count(received.solverResults) == 2 && received.count(received.syncReadies) == 2 &&
               received.count(received.syncTalks) == 2;
    }));
    EXPECT_EQ(received.agentAnnouncements[0].senderID, 1u);
    EXPECT_EQ(received.agentAnnouncements[1].senderID, 2u);
    EXPECT_EQ(received.planTreeInfos[1].stateIDs, (std::vector<int64_t>{5}));
}

TEST_F(AlicaRosCommunicationTest, EdgeBroadcastPublishesOnAlicaEngineTopics)
{
    // Also for the senders: they subscribe to the topics they publish on
    Received ignored;
    // The edge broadcaster collects the messages of each agent from the normal topics
    alicaRosProxy::AlicaRosEdgeBroadcastCommunication communication(ignored.handlers(), _node);
    communication.startCommunication();

    std::atomic<int> count(0);
    auto broadcaster = rclcpp::Node::make_shared("edge_broadcaster");
    auto sub = broadcaster->create_subscription<alica_msgs::msg::PlanTreeInfo>(
            "/AlicaEngine/PlanTreeInfo", rclcpp::QoS(10).reliable(), [&count](const alica_msgs::msg::PlanTreeInfo&) { ++count; });
    _executor.add_node(broadcaster);
    ASSERT_TRUE(waitForMatch(_node, {"/AlicaEngine/PlanTreeInfo"}, 1));

    alica::PlanTreeInfo pti;
    pti.senderID = 1;
    communication.sendPlanTreeInfo(pti);
    EXPECT_TRUE(waitFor([&]() { return count == 1; }));
    _executor.remove_node(broadcaster);
}

TEST_F(AlicaRosCommunicationTest, DestructorWaitsForCallbackOnExecutorOfCaller)
{
    // The executor of the fixture, not the communication, runs the callbacks. The destructor must wait for a callback
    // that is in progress, because the callback uses the handlers of the object.
    Received ignored;
    alicaRosProxy::AlicaRosCommunication sender(ignored.handlers(), _node);
    sender.startCommunication();

    std::atomic<bool> handlerStarted(false), handlerFinished(false);
    alica::AlicaCommunicationHandlers handlers = ignored.handlers();
    handlers.planTreeInfohandler = [&](std::shared_ptr<alica::PlanTreeInfo>) {
        handlerStarted = true;
        std::this_thread::sleep_for(100ms);
        handlerFinished = true;
    };
    auto receiver = std::make_unique<alicaRosProxy::AlicaRosCommunication>(handlers, _node);
    receiver->startCommunication();
    ASSERT_TRUE(waitForMatch(_node, {"/AlicaEngine/PlanTreeInfo"}, 2));

    alica::PlanTreeInfo pti;
    pti.senderID = 1;
    sender.sendPlanTreeInfo(pti);
    ASSERT_TRUE(waitFor([&]() { return handlerStarted.load(); }));

    receiver.reset();
    ASSERT_TRUE(handlerFinished);
}

} // namespace

int main(int argc, char* argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
