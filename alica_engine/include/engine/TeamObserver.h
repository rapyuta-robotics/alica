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

/**
 * The TeamObserver manages communication with the team. Thus it sends and receives PlanTreeInfo messages.
 * Specialized Modules may communicate through other means.
 */
class TeamObserver
{
public:
    TeamObserver(AlicaEngine* ae);
    ~TeamObserver();

    void tick(RunningPlan* root);
    void doBroadCast(const IdGrp& msg) const;

    const std::map<AgentId, std::unique_ptr<SimplePlanTree>>& getTeamPlanTrees() const { return _simplePlanTrees; }

    int successesInPlan(std::size_t parentContextHash, const Plan* plan) const;
    SuccessCollection createSuccessCollection(const Plan* plan) const;
    void updateSuccessCollection(const Plan* p, SuccessCollection& sc);

    void notifyRobotLeftPlan(const AbstractPlan* plan) const;
    void handlePlanTreeInfo(std::shared_ptr<PlanTreeInfo> incoming);
    void close();

private:
    bool updateTeamPlanTrees();
    void cleanOwnSuccessMarks(RunningPlan* root);
    std::unique_ptr<SimplePlanTree> sptFromMessage(AgentId agent, const IdGrp& ids, AlicaTime time) const;

    AlicaEngine* _ae;
    TeamManager& _tm;
    Agent* _me;

    std::mutex _msgQueueMutex;
    mutable std::mutex _successMarkMutex;

    std::map<AgentId, std::unique_ptr<SimplePlanTree>> _simplePlanTrees;
    std::vector<std::pair<std::shared_ptr<PlanTreeInfo>, AlicaTime>> _msgQueue;
};

} /* namespace alica */
