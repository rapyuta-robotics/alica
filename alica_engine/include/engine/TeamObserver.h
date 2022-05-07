#pragma once

#include <engine/Types.h>

#include <engine/AlicaClock.h>

#include <map>
#include <memory>
#include <mutex>

namespace alica
{

class Logger;
class AlicaEngine;
class SuccessCollection;
class TeamManager;
class Agent;
class PlanTreeInfo;
class RunningPlan;
class SimplePlanTree;
class IRoleAssignment;
class PlanRepository;
class IAlicaCommunication;

/**
 * The TeamObserver manages communication with the team. Thus it sends and receives PlanTreeInfo messages.
 * Specialized Modules may communicate through other means.
 */
class TeamObserver
{
public:
    //[[deprecated("It will be removed in the last PR")]]
    TeamObserver(AlicaEngine* ae);
    TeamObserver(Logger& logger, IRoleAssignment& roleAssigment, const bool& maySendMessages, std::shared_ptr<AlicaClock> clock,
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

    void setCommunicator(std::shared_ptr<IAlicaCommunication> communicator);
    void setAlicaClock(std::shared_ptr<AlicaClock> clock);

private:
    bool updateTeamPlanTrees();
    void cleanOwnSuccessMarks(RunningPlan* root);
    std::unique_ptr<SimplePlanTree> sptFromMessage(AgentId agent, const IdGrp& ids, AlicaTime time) const;

    AlicaEngine* _ae;
    Agent* _me;
    Logger& _logger;
    IRoleAssignment& _roleAssignment;
    const bool& _maySendMessages;
    std::shared_ptr<IAlicaCommunication> _communicator{nullptr};
    std::shared_ptr<AlicaClock> _alicaClock;
    const PlanRepository& _planRepository;
    TeamManager& _tm;

    std::mutex _msgQueueMutex;
    mutable std::mutex _successMarkMutex;

    std::map<AgentId, std::unique_ptr<SimplePlanTree>> _simplePlanTrees;
    std::vector<std::pair<std::shared_ptr<PlanTreeInfo>, AlicaTime>> _msgQueue;
};

} /* namespace alica */
