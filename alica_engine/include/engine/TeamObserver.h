#pragma once

#include <engine/Types.h>

#include <engine/AlicaClock.h>

#include <map>
#include <memory>
#include <mutex>

namespace alica
{

class SuccessCollection;
class TeamManager;
class Agent;
struct PlanTreeInfo;
class RunningPlan;
class SimplePlanTree;
class IRoleAssignment;
class PlanRepository;
class IAlicaCommunication;
class ConfigChangeListener;

/**
 * The TeamObserver manages communication with the team. Thus it sends and receives PlanTreeInfo messages.
 * Specialized Modules may communicate through other means.
 */
class TeamObserver
{
public:
    TeamObserver(ConfigChangeListener& configChangeListener, IRoleAssignment& roleAssigment, const IAlicaCommunication& communicator, const AlicaClock& clock,
            const PlanRepository& planRepository, TeamManager& teamManager);
    ~TeamObserver();

    void tick(RunningPlan* root);
    void doBroadCast(const IdGrp& msg) const;

    const std::map<AgentId, std::unique_ptr<SimplePlanTree>>& getTeamPlanTrees() const { return _simplePlanTrees; }

    int successesInPlan(const Plan* plan) const;
    SuccessCollection createSuccessCollection(const Plan* plan) const;
    void updateSuccessCollection(const Plan* p, SuccessCollection& sc);

    void notifyRobotLeftPlan(const AbstractPlan* plan) const;
    void handlePlanTreeInfo(std::shared_ptr<PlanTreeInfo> incoming);
    void close();
    void reload(const YAML::Node& config);

private:
    static constexpr const char* LOGNAME = "TeamObserver";

    bool updateTeamPlanTrees();
    void cleanOwnSuccessMarks(RunningPlan* root);
    std::unique_ptr<SimplePlanTree> sptFromMessage(AgentId agent, const IdGrp& ids, AlicaTime time) const;

    IRoleAssignment& _roleAssignment;
    const IAlicaCommunication& _communicator;
    const AlicaClock& _clock;
    const PlanRepository& _planRepository;
    TeamManager& _tm;
    Agent* _me;
    bool _maySendMessages;
    std::mutex _msgQueueMutex;
    mutable std::mutex _successMarkMutex;

    std::map<AgentId, std::unique_ptr<SimplePlanTree>> _simplePlanTrees;
    std::vector<std::pair<std::shared_ptr<PlanTreeInfo>, AlicaTime>> _msgQueue;
};

} /* namespace alica */
